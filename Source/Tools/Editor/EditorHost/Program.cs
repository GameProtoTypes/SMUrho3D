//
// Copyright (c) 2018 Rokas Kupstys
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using File = System.IO.File;
using Urho3DNet;

namespace EditorHost
{
    [StructLayout(LayoutKind.Sequential)]
    internal struct DomainManagerInterface
    {
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate bool LoadPluginDelegate(IntPtr handle, string path);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void SetReloadingDelegate(IntPtr handle, bool reloading);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate bool GetReloadingDelegate(IntPtr handle);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate bool IsPluginDelegate(IntPtr handle, string path);

        public IntPtr handle;

        [MarshalAs(UnmanagedType.FunctionPtr)] public LoadPluginDelegate loadPlugin;
        [MarshalAs(UnmanagedType.FunctionPtr)] public SetReloadingDelegate setReloading;
        [MarshalAs(UnmanagedType.FunctionPtr)] public GetReloadingDelegate getReloading;
        [MarshalAs(UnmanagedType.FunctionPtr)] public IsPluginDelegate isPlugin;
    }

    /// This class runs in a context of reloadable appdomain.
    internal class DomainManager : MarshalByRefObject, IDisposable
    {
        private DomainManagerInterface _interface;
        private Context _context;
        private List<PluginApplication> _plugins = new List<PluginApplication>();

        public bool Reloading { get; protected set; }

        public DomainManager(IntPtr contextPtr)
        {
            _context = Context.wrap(contextPtr, true);
            _loadPluginDelegateRef = NLoadPlugin;
            _setReloadingDelegateRef = NSetReloading;
            _getReloadingDelegateRef = NGetReloading;
            _isPluginDelegateRef = NIsPlugin;
            _interface = new DomainManagerInterface
            {
                handle = GCHandle.ToIntPtr(GCHandle.Alloc(this)),
                loadPlugin = _loadPluginDelegateRef,
                setReloading = _setReloadingDelegateRef,
                getReloading = _getReloadingDelegateRef,
                isPlugin = _isPluginDelegateRef,
            };
            SetManagedRuntimeInterface(_interface);
            Urho3DPINVOKE.DelegateRegistry.RefreshDelegatePointers();

            AppDomain.CurrentDomain.ReflectionOnlyAssemblyResolve += (sender, args) =>
            {
                return Assembly.ReflectionOnlyLoadFrom(
                    Path.Combine(Path.GetDirectoryName(args.RequestingAssembly.Location),
                    args.Name.Substring(0, args.Name.IndexOf(',')) + ".dll"));
            };
        }

        public void Dispose()
        {
            foreach (var plugin in _plugins)
            {
                plugin.Unload();
                plugin.Dispose();
            }

            _plugins.Clear();

            GCHandle.FromIntPtr(_interface.handle).Free();
            _interface.handle = IntPtr.Zero;
            _context = null;
            GC.Collect(GC.MaxGeneration);
            GC.WaitForPendingFinalizers();
        }

        /// Returns a versioned path.
        private string GetNewPluginPath(string path)
        {
            var fileDir = Path.GetDirectoryName(path);
            var fileName = Path.GetFileNameWithoutExtension(path);
            for (int i = 0; ; i++)
            {
                path = $"{fileDir}{Path.DirectorySeparatorChar}{fileName}{i}.dll";
                if (!File.Exists(path))
                    return path;
            }
        }

        private string VersionFile(string path)
        {
            if (path == null)
                throw new ArgumentException($"{nameof(path)} may not be null.");

            var pdbPath = Path.ChangeExtension(path, "pdb");
            var versionedPath = GetNewPluginPath(path);
            var versionedPdbPath = Path.ChangeExtension(versionedPath, "pdb");
            Debug.Assert(versionedPath != null, $"{nameof(versionedPath)} != null");
            File.Copy(path, versionedPath, true);

            if (File.Exists(pdbPath))
            {
                File.Copy(pdbPath, versionedPdbPath, true);

                // Update .pdb path in a newly copied file
                var pathBytes = Encoding.ASCII.GetBytes(Path.GetFileName(pdbPath)); // Does this work with i18n paths?
                var dllBytes = File.ReadAllBytes(versionedPath);
                int i;
                for (i = 0; i < dllBytes.Length; i++)
                {
                    if (dllBytes.Skip(i).Take(pathBytes.Length).SequenceEqual(pathBytes)) // I know its slow ¯\_(ツ)_/¯
                    {
                        if (dllBytes[i + pathBytes.Length] != 0) // Check for null terminator
                            continue;

                        while (dllBytes[--i] != 0)
                        {
                            // We found just a file name. Time to walk back to find a start of the string. This is
                            // required because dll is executing from non-original directory and pdb path points
                            // somewhere else we can not predict.
                        }

                        i++;

                        // Copy full pdb path
                        var newPathBytes = Encoding.ASCII.GetBytes(versionedPdbPath);
                        newPathBytes.CopyTo(dllBytes, i);
                        dllBytes[i + newPathBytes.Length] = 0;
                        break;
                    }
                }

                if (i == dllBytes.Length)
                    return null;

                File.WriteAllBytes(versionedPath, dllBytes);
            }

            return versionedPath;
        }

        public bool LoadPlugin(string path)
        {
            if (!File.Exists(path) && !path.EndsWith(".dll"))
                return false;

            path = VersionFile(path);
            if (path == null)
                return false;

            Assembly assembly;
            try
            {
                assembly = Assembly.LoadFile(path);
            }
            catch (Exception)
            {
                return false;
            }

            Type pluginType = null;
            foreach (var type in assembly.GetTypes())
            {
                if (!type.IsClass)
                    continue;

                if (type.BaseType == typeof(PluginApplication))
                {
                    pluginType = type;
                    break;
                }
            }

            if (pluginType == null)
                return false;

            var plugin = Activator.CreateInstance(pluginType, _context) as PluginApplication;
            if (plugin is null)
                return false;

            plugin.Load();
            _plugins.Add(plugin);

            return true;
        }

        public bool IsPlugin(string path)
        {
            var pluginBaseName = typeof(PluginApplication).FullName;
            try
            {
                var assembly = Assembly.ReflectionOnlyLoadFrom(path);
                var types = assembly.GetTypes();
                return types.Any(type => type.BaseType?.FullName == pluginBaseName);
            }
            catch (Exception _)
            {
                return false;
            }
            return false;
        }

        #region Interop

        [DllImport("libEditor", CallingConvention = CallingConvention.Cdecl)]
        private static extern void SetManagedRuntimeInterface(in DomainManagerInterface @interface);

        private readonly DomainManagerInterface.LoadPluginDelegate _loadPluginDelegateRef;
        private static bool NLoadPlugin(IntPtr handle, [MarshalAs(UnmanagedType.LPUTF8Str)]string path)
        {
            return ((DomainManager) GCHandle.FromIntPtr(handle).Target).LoadPlugin(path);
        }

        private readonly DomainManagerInterface.SetReloadingDelegate _setReloadingDelegateRef;
        private static void NSetReloading(IntPtr handle, bool reloading)
        {
            ((DomainManager) GCHandle.FromIntPtr(handle).Target).Reloading = reloading;
        }

        private readonly DomainManagerInterface.GetReloadingDelegate _getReloadingDelegateRef;
        private static bool NGetReloading(IntPtr handle)
        {
            return ((DomainManager) GCHandle.FromIntPtr(handle).Target).Reloading;
        }

        private readonly DomainManagerInterface.IsPluginDelegate _isPluginDelegateRef;
        private static bool NIsPlugin(IntPtr handle, [MarshalAs(UnmanagedType.LPUTF8Str)]string path)
        {
            return ((DomainManager)GCHandle.FromIntPtr(handle).Target).IsPlugin(path);
        }

        #endregion
    }

    internal class Program
    {
        private HandleRef _contextPtr;
        private int Version { get; set; }
        private Thread _mainThread;

        private AppDomain CreateDomain()
        {
            var domain = AppDomain.CreateDomain(AppDomain.CurrentDomain.FriendlyName.Replace(".", $"{++Version}."));
            return domain;
        }

        private void CreateMainThread(string[] args)
        {
            var argc = args.Length + 1;                 // args + executable path
            var argv = new string[args.Length + 2];     // args + executable path + null
            var threadStarted = false;

            // As per C spec first argv item must be a program name and item after the last one must be a null pointer.
            argv[0] = new Uri(Assembly.GetExecutingAssembly().CodeBase).LocalPath;
            args.CopyTo(argv, 1);

            _mainThread = new Thread(() =>
            {
                ParseArgumentsC(argc, argv);
                using (var context = new Context())
                {
                    _contextPtr = Context.getCPtr(context);
                    using (var editor = Application.wrap(CreateEditorApplication(_contextPtr.Handle), true))
                    {
                        threadStarted = true;
                        Environment.ExitCode = editor.Run();
                    }
                }
            });
            _mainThread.Start();
            while (!threadStarted)
                Thread.Sleep(60);
        }

        private void Run(string[] args)
        {
            var managerType = typeof(DomainManager);
            if (managerType.FullName is null)
                throw new ArgumentException("DomainManager.FullName is null");

            CreateMainThread(args);

            while (_mainThread.IsAlive)
            {
                var executionDomain = CreateDomain();
                var manager = executionDomain.CreateInstanceAndUnwrap(managerType.Assembly.FullName,
                    managerType.FullName, false, BindingFlags.Default, null,
                    new object[]{_contextPtr.Handle}, null, null) as DomainManager;

                if (manager is null)
                    throw new NullReferenceException($"Failed creating {managerType.FullName}.");

                while (!manager.Reloading && _mainThread.IsAlive)
                    Thread.Sleep(100);

                manager.Dispose();

                for (;;)
                {
                    try
                    {
                        AppDomain.Unload(executionDomain);
                        break;
                    }
                    catch (CannotUnloadAppDomainException)
                    {
                        Thread.Sleep(1000);
                    }
                }
            }
        }

        public static void Main(string[] args)
        {
            new Program().Run(args);
        }

        [DllImport("libEditor", CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr CreateEditorApplication(IntPtr context);

        [DllImport("libEditor", CallingConvention = CallingConvention.Cdecl)]
        private static extern void ParseArgumentsC(int argc,
            [MarshalAs(UnmanagedType.LPArray, ArraySubType = UnmanagedType.LPStr)]string[] argv);
    }
}
