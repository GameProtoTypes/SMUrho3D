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


#include <Urho3D/Urho3DAll.h>
#include <Toolbox/SystemUI/AttributeInspector.h>
#include "Project.h"
#include "Plugins/PluginManagerNative.h"

using namespace std::placeholders;

namespace Urho3D
{

class Tab;
class SceneTab;
class AssetConverter;

class Editor : public Application
{
    URHO3D_OBJECT(Editor, Application);
public:
    /// Construct.
    explicit Editor(Context* context);
    /// Set up editor application.
    void Setup() override;
    /// Initialize editor application.
    void Start() override;
    /// Tear down editor application.
    void Stop() override;

    /// Renders UI elements.
    void OnUpdate(VariantMap& args);
    /// Renders menu bar at the top of the screen.
    void RenderMenuBar();
    /// Create a new tab of specified type.
    template<typename T> T* CreateTab() { return (T*)CreateTab(T::GetTypeStatic()); }
    /// Create a new tab of specified type.
    Tab* CreateTab(StringHash type);
    /// Return active scene tab.
    Tab* GetActiveTab() { return activeTab_; }
    /// Return currently open scene tabs.
    const Vector<SharedPtr<Tab>>& GetSceneViews() const { return tabs_; }
    /// Return a map of names and type hashes from specified category.
    StringVector GetObjectsByCategory(const String& category);
    /// Get absolute path of `resourceName`. If it is empty, use `defaultResult`. If no resource is found then save file
    /// dialog will be invoked for selecting a new path.
    String GetResourceAbsolutePath(const String& resourceName, const String& defaultResult, const char* patterns,
        const String& dialogTitle);
    /// Returns a list of open content tabs/docks/windows. This list does not include utility docks/tabs/windows.
    const Vector<SharedPtr<Tab>>& GetContentTabs() const { return tabs_; }
    /// Opens project or creates new one.
    Project* OpenProject(const String& projectPath);
    /// Close current project.
    void CloseProject();
    /// Return path containing data directories of engine.
    const String& GetCoreResourcePrefixPath() const { return coreResourcePrefixPath_; }
    /// Load default tab layout.
    void LoadDefaultLayout();

protected:
    /// Process console commands.
    void OnConsoleCommand(VariantMap& args);

    /// List of active scene tabs.
    Vector<SharedPtr<Tab>> tabs_;
    /// Last focused scene tab.
    WeakPtr<Tab> activeTab_;
    /// Prefix path of CoreData and EditorData.
    String coreResourcePrefixPath_;
    /// Currently loaded project.
    SharedPtr<Project> project_;
#if URHO3D_PLUGINS_NATIVE
    /// Native plugin manager.
    PluginManagerNative pluginsNative_;
#endif
};

}
