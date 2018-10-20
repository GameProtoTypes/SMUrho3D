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

#pragma once


#if URHO3D_PLUGINS

#include <atomic>
#include <cr/cr.h>
#include <Urho3D/Core/Object.h>
#include <Urho3D/IO/FileWatcher.h>

namespace Urho3D
{

/// Enumeration describing plugin file path status.
enum PluginType
{
    /// Not a valid plugin.
    PLUGIN_INVALID,
    /// A native plugin.
    PLUGIN_NATIVE,
    /// A managed plugin.
    PLUGIN_MANAGED,
};

enum class ReloadStatus
{
    None,
    ReloadRequest,
    Reloading,
};

struct DomainManagerInterface
{
    void* handle;
    bool(*LoadPlugin)(void* handle, const char* path);
    void(*SetReloadStatus)(void* handle, ReloadStatus status);
    ReloadStatus(*GetReloadStatus)(void* handle);
    bool(*IsPlugin)(void* handle, const char* path);
};

class Plugin : public Object
{
    URHO3D_OBJECT(Plugin, Object);
public:
    explicit Plugin(Context* context);

    /// Returns type of the plugin.
    PluginType GetPluginType() const { return type_; }
    /// Returns file name of plugin.
    String GetName() const { return name_; }

protected:
    /// Unload plugin.
    bool Unload();

    /// Base plugin file name.
    String name_;
    /// Path to plugin dynamic library file.
    String path_;
    /// Type of plugin (invalid/native/managed).
    PluginType type_ = PLUGIN_INVALID;
    /// Context of native plugin. Not initialized for managed plugins.
    cr_plugin nativeContext_{};
    /// Flag indicating that plugin should unload on the end of the frame.
    bool unloading_ = false;
    /// Last modification time.
    unsigned mtime_;

    friend class PluginManager;
};

class PluginManager : public Object
{
    URHO3D_OBJECT(PluginManager, Object);
public:
    /// Construct.
    explicit PluginManager(Context* context);
    /// Unload all plugins an destruct.
    ~PluginManager();
    /// Load a plugin and return true if succeeded.
    virtual Plugin* Load(const String& name);
    /// Unload a plugin and return true if succeeded.
    virtual void Unload(Plugin* plugin);
    /// Returns a loaded plugin with specified name.
    Plugin* GetPlugin(const String& name);
    /// Returns a vector containing all loaded plugins.
    const Vector<SharedPtr<Plugin>>& GetPlugins() const { return plugins_; }
    /// Tick native plugins.
    void OnEndFrame();
    /// Converts relative or absolute plugin path to universal plugin name. Returns empty string on failure.
    static String PathToName(const String& path);
    /// Checks specified file and recognizes it's plugin type.
    static PluginType GetPluginType(const String& path);

protected:
    /// Delete temporary files from binary directory.
    void CleanUp(String directory = String::EMPTY);
    /// Converts name to a full plugin file path. Returns empty string on error.
    String NameToPath(const String& name) const;

    /// Loaded plugins.
    Vector<SharedPtr<Plugin>> plugins_;

    friend class Plugin;
};

}
#endif
