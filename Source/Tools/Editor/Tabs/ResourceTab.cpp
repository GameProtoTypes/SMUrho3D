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

#include <IconFontCppHeaders/IconsFontAwesome.h>
#include <Toolbox/IO/ContentUtilities.h>
#include "Tabs/Scene/SceneTab.h"
#include "Tabs/UI/UITab.h"
#include "Editor.h"
#include "ResourceTab.h"

namespace Urho3D
{

static HashMap<ContentType, String> contentToTabType{
    {CTYPE_SCENE, "SceneTab"},
    {CTYPE_UILAYOUT, "UITab"},
};

unsigned MakeHash(ContentType value)
{
    return (unsigned)value;
}

ResourceTab::ResourceTab(Context* context)
    : Tab(context)
{
    isUtility_ = true;
    SetTitle("Resources");

    SubscribeToEvent(E_INSPECTORLOCATERESOURCE, [&](StringHash, VariantMap& args) {
        auto resourceName = args[InspectorLocateResource::P_NAME].GetString();
        resourcePath_ = GetPath(resourceName);
        resourceSelection_ = GetFileNameAndExtension(resourceName);
        flags_ |= RBF_SCROLL_TO_CURRENT;
    });
    SubscribeToEvent(E_RESOURCEBROWSERRENAME, [&](StringHash, VariantMap& args) {
        using namespace ResourceBrowserRename;
        auto* project = GetSubsystem<Project>();
        auto sourceName = project->GetResourcePath() + args[P_FROM].GetString();
        auto destName = project->GetResourcePath() + args[P_TO].GetString();

        if (GetCache()->RenameResource(sourceName, destName))
            resourceSelection_ = GetFileNameAndExtension(destName);
        else
            URHO3D_LOGERRORF("Renaming '%s' to '%s' failed.", sourceName.CString(), destName.CString());
    });
    SubscribeToEvent(E_RESOURCEBROWSERDELETE, [&](StringHash, VariantMap& args) {
        using namespace ResourceBrowserDelete;
        auto* project = GetSubsystem<Project>();
        auto fileName = project->GetResourcePath() + args[P_NAME].GetString();
        if (GetFileSystem()->FileExists(fileName))
            GetFileSystem()->Delete(fileName);
        else if (GetFileSystem()->DirExists(fileName))
            GetFileSystem()->RemoveDir(fileName, true);
    });
}

bool ResourceTab::RenderWindowContent()
{
    auto action = ResourceBrowserWidget(resourcePath_, resourceSelection_, flags_);
    if (action == RBR_ITEM_OPEN)
    {
        String selected = resourcePath_ + resourceSelection_;
        auto it = contentToTabType.Find(GetContentType(selected));
        if (it != contentToTabType.End())
        {
            auto* tab = GetSubsystem<Editor>()->CreateTab(it->second_);
            tab->AutoPlace();
            tab->LoadResource(selected);
        }
    }
    else if (action == RBR_ITEM_CONTEXT_MENU)
        ui::OpenPopup("Resource Context Menu");

    flags_ = RBF_NONE;

    if (ui::BeginPopup("Resource Context Menu"))
    {
        if (ui::BeginMenu("Create"))
        {
            if (ui::MenuItem(ICON_FA_FOLDER " Folder"))
            {
                String newFolderName("New Folder");
                String path = GetNewResourcePath(resourcePath_ + newFolderName);
                if (GetFileSystem()->CreateDir(path))
                {
                    flags_ |= RBF_RENAME_CURRENT | RBF_SCROLL_TO_CURRENT;
                    resourceSelection_ = newFolderName;
                }
                else
                    URHO3D_LOGERRORF("Failed creating folder '%s'.", path.CString());
            }

            if (ui::MenuItem("Scene"))
            {
                auto path = GetNewResourcePath(resourcePath_ + "New Scene.scene");
                GetFileSystem()->CreateDirsRecursive(GetPath(path));

                SharedPtr<Scene> scene(new Scene(context_));
                scene->CreateComponent<Octree>();
                File file(context_, path, FILE_WRITE);
                if (file.IsOpen())
                {
                    scene->SaveYAML(file);
                    flags_ |= RBF_RENAME_CURRENT | RBF_SCROLL_TO_CURRENT;
                    resourceSelection_ = GetFileNameAndExtension(path);
                }
                else
                    URHO3D_LOGERRORF("Failed opening file '%s'.", path.CString());
            }

            if (ui::MenuItem("UI Layout"))
            {
                auto path = GetNewResourcePath(resourcePath_ + "New UI Layout.xml");
                GetFileSystem()->CreateDirsRecursive(GetPath(path));

                SharedPtr<UIElement> scene(new UIElement(context_));
                XMLFile layout(context_);
                auto root = layout.GetOrCreateRoot("element");
                if (scene->SaveXML(root) && layout.SaveFile(path))
                {
                    flags_ |= RBF_RENAME_CURRENT | RBF_SCROLL_TO_CURRENT;
                    resourceSelection_ = GetFileNameAndExtension(path);
                }
                else
                    URHO3D_LOGERRORF("Failed opening file '%s'.", path.CString());
            }

            ui::EndMenu();
        }

        if (ui::MenuItem("Rename", "F2"))
            flags_ |= RBF_RENAME_CURRENT;

        if (ui::MenuItem("Delete", "Del"))
            flags_ |= RBF_DELETE_CURRENT;

        ui::EndPopup();
    }

    return true;
}

String ResourceTab::GetNewResourcePath(const String& name)
{
    auto* project = GetSubsystem<Project>();
    if (!GetFileSystem()->FileExists(project->GetResourcePath() + name))
        return project->GetResourcePath() + name;

    auto basePath = GetPath(name);
    auto baseName = GetFileName(name);
    auto ext = GetExtension(name, false);

    for (auto i = 1; i < M_MAX_INT; i++)
    {
        auto newName = project->GetResourcePath() + ToString("%s%s %d%s", basePath.CString(), baseName.CString(), i, ext.CString());
        if (!GetFileSystem()->FileExists(newName))
            return newName;
    }

    std::abort();
}

}
