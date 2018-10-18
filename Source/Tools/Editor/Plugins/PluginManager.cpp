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

#if URHO3D_PLUGINS

#define CR_HOST

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Core/Thread.h>
#include <Urho3D/Engine/PluginApplication.h>
#include <Urho3D/IO/File.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/IO/Log.h>
#include <EditorEvents.h>
#include "Editor.h"
#include "PluginManager.h"
#include "EditorEventsPrivate.h"


namespace Urho3D
{

#if __linux__
static const char* platformDynamicLibrarySuffix = ".so";
#elif _WIN32
static const char* platformDynamicLibrarySuffix = ".dll";
#elif __APPLE__
static const char* platformDynamicLibrarySuffix = ".dylib";
#else
#   error Unsupported platform.
#endif

#if URHO3D_CSHARP && URHO3D_PLUGINS

struct DomainManagerInterface
{
    void* handle;
    bool(*LoadPlugin)(void* handle, const char* path);
    void(*SetReloading)(void* handle, bool reloading);
    bool(*GetReloading)(void* handle);
};

static Mutex managedInterfaceLock;
static DomainManagerInterface managedInterface_;

/// A native thread that will run editor when it is started from a .net loader executable.
class EditorMainThread : public Thread
{
public:
    EditorMainThread()
    {
        context_ = new Urho3D::Context();
    }

    void ThreadFunction() override
    {
        Thread::SetMainThread();
        Urho3D::SharedPtr<Editor> application(new Editor(context_));
        int exitCode = application->Run();
        application = nullptr;
        context_ = nullptr;
        exit(exitCode);
    }

    Urho3D::SharedPtr<Urho3D::Context> context_;
};

/// Kick off editor main thread. Should be called only once.
extern "C" URHO3D_EXPORT_API Context* StartMainThread(int argc, char** argv)
{
    Urho3D::ParseArguments(argc, argv);
    static EditorMainThread mainThread;
    mainThread.Run();
    return mainThread.context_;
}

/// Update a pointer to a struct that facilitates interop between native code and a .net runtime manager object. Will be called every time .net code is reloaded.
extern "C" URHO3D_EXPORT_API void SetManagedRuntimeInterface(DomainManagerInterface* managedInterface)
{
    MutexLock lock(managedInterfaceLock);
    managedInterface_ = *managedInterface;
}

#endif

Plugin::Plugin(Context* context)
    : Object(context)
{
}

PluginManager::PluginManager(Context* context)
    : Object(context)
{
    CleanUp();
    SubscribeToEvent(E_ENDFRAME, [this](StringHash, VariantMap&) { OnEndFrame(); });
    SubscribeToEvent(E_SIMULATIONSTART, [this](StringHash, VariantMap&) {
        for (auto& plugin : plugins_)
        {
            if (plugin->nativeContext_.userdata != nullptr && plugin->nativeContext_.userdata != context_)
                reinterpret_cast<PluginApplication*>(plugin->nativeContext_.userdata)->Start();
        }
    });
    SubscribeToEvent(E_SIMULATIONSTOP, [this](StringHash, VariantMap&) {
        for (auto& plugin : plugins_)
        {
            if (plugin->nativeContext_.userdata != nullptr && plugin->nativeContext_.userdata != context_)
                reinterpret_cast<PluginApplication*>(plugin->nativeContext_.userdata)->Stop();
        }
    });
}

PluginType PluginManager::GetPluginType(const String& path)
{
    File file(context_);
    if (!file.Open(path, FILE_READ))
        return PLUGIN_INVALID;

    // This function implements a naive check for plugin validity. Proper check would parse executable headers and look
    // for relevant exported function names.

#if __linux__
    // ELF magic
    if (path.EndsWith(".so"))
    {
        if (file.ReadUInt() == 0x464C457F)
        {
            file.Seek(0);
            String buf{ };
            buf.Resize(file.GetSize());
            file.Read(&buf[0], file.GetSize());
            auto pos = buf.Find("cr_main");
            // Function names are preceeded with 0 in elf files.
            if (pos != String::NPOS && buf[pos - 1] == 0)
                return PLUGIN_NATIVE;
        }
    }
#endif
    file.Seek(0);
    if (path.EndsWith(".dll"))
    {
        if (file.ReadShort() == 0x5A4D)
        {
#if _WIN32
            // But only on windows we check if PE file is a native plugin
            file.Seek(0);
            String buf{};
            buf.Resize(file.GetSize());
            file.Read(&buf[0], file.GetSize());
            auto pos = buf.Find("cr_main");
            // Function names are preceeded with 2 byte hint which is preceeded with 0 in PE files.
            if (pos != String::NPOS && buf[pos - 3] == 0)
                return PLUGIN_NATIVE;
#endif
            // PE files are handled on all platforms because managed executables are PE files.
            file.Seek(0x3C);
            auto e_lfanew = file.ReadUInt();
#if URHO3D_64BIT
            const auto netMetadataRvaOffset = 0xF8;
#else
            const auto netMetadataRvaOffset = 0xE8;
#endif
            file.Seek(e_lfanew + netMetadataRvaOffset);  // Seek to .net metadata directory rva

            if (file.ReadUInt() != 0)
                return PLUGIN_MANAGED;
        }
    }

    if (path.EndsWith(".dylib"))
    {
        // TODO: MachO file support.
    }

    return PLUGIN_INVALID;
}

Plugin* PluginManager::Load(const String& name)
{
#if URHO3D_PLUGINS
    if (Plugin* loaded = GetPlugin(name))
        return loaded;

    CleanUp();

    String pluginPath = NameToPath(name);
    if (pluginPath.Empty())
        return nullptr;

    SharedPtr<Plugin> plugin(new Plugin(context_));
    plugin->type_ = GetPluginType(pluginPath);

    if (plugin->type_ == PLUGIN_NATIVE)
    {
        if (cr_plugin_load(plugin->nativeContext_, pluginPath.CString()))
        {
            plugin->nativeContext_.userdata = context_;
            plugin->name_ = name;
            plugin->path_ = pluginPath;
            plugins_.Push(plugin);
            return plugin.Get();
        }
        else
            URHO3D_LOGWARNINGF("Failed loading native plugin \"%s\".", name.CString());
    }
#if URHO3D_CSHARP
    else if (plugin->type_ == PLUGIN_MANAGED)
    {
        if (managedInterface_.LoadPlugin(managedInterface_.handle, pluginPath.CString()))
        {
            plugin->name_ = name;
            plugin->path_ = pluginPath;
            plugins_.Push(plugin);
            return plugin.Get();
        }
    }
#endif
#endif
    return nullptr;
}

void PluginManager::Unload(Plugin* plugin)
{
    if (plugin == nullptr)
        return;

    auto it = plugins_.Find(SharedPtr<Plugin>(plugin));
    if (it == plugins_.End())
    {
        URHO3D_LOGERRORF("Plugin %s was never loaded.", plugin->name_.CString());
        return;
    }

#if URHO3D_WITH_MONO
    // Managed plugin unloading/reloading is not supported due to https://github.com/mono/mono/issues/11170
    if (plugin->type_ == PLUGIN_MANAGED)
    {
        URHO3D_LOGERROR("Unloading of managed plugins is not supported due to mono bug 11170.");
        return;
    }
#endif

    plugin->unloading_ = true;
}

void PluginManager::OnEndFrame()
{
#if URHO3D_PLUGINS
    for (auto it = plugins_.Begin(); it != plugins_.End();)
    {
        Plugin* plugin = it->Get();

        if (plugin->unloading_)
        {
            SendEvent(E_EDITORUSERCODERELOADSTART);
            if (plugin->type_ == PLUGIN_NATIVE)
            {
                cr_plugin_close(plugin->nativeContext_);
                plugin->nativeContext_.userdata = nullptr;
            }
#if URHO3D_CSHARP
            else if (plugin->type_ == PLUGIN_MANAGED)
            {
                // Managed plugin unloading requires to tear down entire AppDomain and recreate it. Instruct main .net thread to
                // do that and wait.
                managedInterfaceLock.Acquire();
                managedInterface_.SetReloading(managedInterface_.handle, true);
                managedInterface_.handle = nullptr;
                managedInterfaceLock.Release();

                // Wait for AppDomain reload.
                for (;;)
                {
                    MutexLock lock(managedInterfaceLock);
                    if (managedInterface_.handle != nullptr)
                        break;                          // Reloading is done.
                    Time::Sleep(30);
                }

                // Now load back all managed plugins except this one.
                for (auto& plug : plugins_)
                {
                    if (plug == plugin || plug->type_ == PLUGIN_NATIVE)
                        continue;
                    managedInterface_.LoadPlugin(managedInterface_.handle, plug->path_.CString());
                }
            }
#endif
            SendEvent(E_EDITORUSERCODERELOADEND);
            URHO3D_LOGINFOF("Plugin %s was unloaded.", plugin->name_.CString());
            it = plugins_.Erase(it);
        }
        else if (plugin->type_ == PLUGIN_NATIVE && plugin->nativeContext_.userdata)
        {
            bool reloading = cr_plugin_changed(plugin->nativeContext_);
            if (reloading)
                SendEvent(E_EDITORUSERCODERELOADSTART);

            if (cr_plugin_update(plugin->nativeContext_) != 0)
            {
                URHO3D_LOGERRORF("Processing plugin \"%s\" failed and it was unloaded.",
                    GetFileNameAndExtension(plugin->name_).CString());
                cr_plugin_close(plugin->nativeContext_);
                plugin->nativeContext_.userdata = nullptr;
                continue;
            }

            if (reloading)
            {
                SendEvent(E_EDITORUSERCODERELOADEND);
                if (plugin->nativeContext_.userdata != nullptr)
                {
                    URHO3D_LOGINFOF("Loaded plugin \"%s\" version %d.",
                        GetFileNameAndExtension(plugin->name_).CString(), plugin->nativeContext_.version);
                }
            }

            it++;
        }
        else
            it++;
    }
#endif
}

void PluginManager::CleanUp(String directory)
{
    if (directory.Empty())
        directory = GetFileSystem()->GetProgramDir();

    if (!GetFileSystem()->DirExists(directory))
        return;

    StringVector files;
    GetFileSystem()->ScanDir(files, directory, "*.*", SCAN_FILES, false);

    for (const String& file : files)
    {
        bool possiblyPlugin = false;
#if __linux__
        possiblyPlugin |= file.EndsWith(".so");
#endif
#if __APPLE__
        possiblyPlugin |= file.EndsWith(".dylib");
#endif
        possiblyPlugin |= file.EndsWith(".dll");

        if (possiblyPlugin)
        {
            String name = GetFileName(file);
            if (IsDigit(static_cast<unsigned int>(name.Back())))
                GetFileSystem()->Delete(ToString("%s/%s", directory.CString(), file.CString()));
        }
    }
}

Plugin* PluginManager::GetPlugin(const String& name)
{
    for (auto it = plugins_.Begin(); it != plugins_.End(); it++)
    {
        if (it->Get()->name_ == name)
            return it->Get();
    }
    return nullptr;
}

String PluginManager::NameToPath(const String& name) const
{
    FileSystem* fs = GetFileSystem();
    String result;

#if __linux__ || __APPLE__
    result = ToString("%slib%s%s", fs->GetProgramDir().CString(), name.CString(), platformDynamicLibrarySuffix);
    if (fs->FileExists(result))
        return result;
#endif

#if !_WIN32
    result = ToString("%s%s%s", fs->GetProgramDir().CString(), name.CString(), ".dll");
    if (fs->FileExists(result))
        return result;
#endif

    result = ToString("%s%s%s", fs->GetProgramDir().CString(), name.CString(), platformDynamicLibrarySuffix);
    if (fs->FileExists(result))
        return result;

    return String::EMPTY;
}

String PluginManager::PathToName(const String& path)
{
    if (path.EndsWith(platformDynamicLibrarySuffix))
    {
        String name = GetFileName(path);
#if __linux__ || __APPLE__
        if (name.StartsWith("lib"))
            name = name.Substring(3);
        return name;
#endif
    }
    else if (path.EndsWith(".dll"))
        return GetFileName(path);
    return String::EMPTY;
}

}

#endif
