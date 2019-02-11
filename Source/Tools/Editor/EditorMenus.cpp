//
// Copyright (c) 2017-2019 Rokas Kupstys.
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

#include <IconFontCppHeaders/IconsFontAwesome5.h>
#include <ImGui/imgui.h>
#include <ImGui/imgui_stdlib.h>
#include <nativefiledialog/nfd.h>
#include <Toolbox/SystemUI/Widgets.h>
#include <Urho3D/Engine/EngineDefs.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/SystemUI/SystemUI.h>
#include "Tabs/Scene/SceneTab.h"
#include "Tabs/PreviewTab.h"
#include "Editor.h"
#include "EditorEvents.h"


using namespace ui::litterals;


namespace Urho3D
{

void Editor::RenderMenuBar()
{
    if (ui::BeginMainMenuBar())
    {
        if (ui::BeginMenu("File"))
        {
            if (project_.NotNull())
            {
                if (ui::MenuItem("Save Project"))
                {
                    for (auto& tab : tabs_)
                        tab->SaveResource();
                    project_->SaveProject();
                }
            }

            if (ui::MenuItem("Open/Create Project"))
            {
                nfdchar_t* projectDir = nullptr;
                if (NFD_PickFolder("", &projectDir) == NFD_OKAY)
                {
                    OpenProject(projectDir);
                    NFD_FreePath(projectDir);
                }
            }

            ui::Separator();

            if (project_.NotNull())
            {
                if (ui::MenuItem("Close Project"))
                {
                    CloseProject();
                }
            }

            if (ui::MenuItem("Exit"))
                engine_->Exit();

            ui::EndMenu();
        }
        if (project_.NotNull())
        {
            if (ui::BeginMenu("View"))
            {
                for (auto& tab : tabs_)
                {
                    if (tab->IsUtility())
                    {
                        // Tabs that can not be closed permanently
                        auto open = tab->IsOpen();
                        if (ui::MenuItem(tab->GetUniqueTitle().CString(), nullptr, &open))
                            tab->SetOpen(open);
                    }
                }
                ui::EndMenu();
            }

            if (ui::BeginMenu("Project"))
            {
                RenderProjectMenu();
                ui::EndMenu();
            }

#if URHO3D_PROFILING
            if (ui::BeginMenu("Tools"))
            {
                if (ui::MenuItem("Profiler"))
                {
                    GetFileSystem()->SystemSpawn(GetFileSystem()->GetProgramDir() + "Profiler"
#if _WIN32
                        ".exe"
#endif
                        , {});
                }
                ui::EndMenu();
            }
#endif
        }

        SendEvent(E_EDITORAPPLICATIONMENU);

        // Scene simulation buttons.
        if (project_.NotNull())
        {
            // Copied from ToolbarButton()
            auto& g = *ui::GetCurrentContext();
            float dimension = g.FontBaseSize + g.Style.FramePadding.y * 2.0f;
            ui::SetCursorScreenPos({ui::GetIO().DisplaySize.x / 2 - dimension * 4 / 2, ui::GetCursorScreenPos().y});
            if (auto* previewTab = GetTab<PreviewTab>())
                previewTab->RenderButtons();
        }

        ui::EndMainMenuBar();
    }
}

void Editor::RenderProjectMenu()
{
#if URHO3D_PLUGINS
    if (ui::BeginMenu("Plugins"))
    {
        ui::PushID("Plugins");
        const StringVector& pluginNames = project_->GetPlugins()->GetPluginNames();
        if (pluginNames.Size() == 0)
        {
            ui::TextUnformatted("No available files.");
            ui::SetHelpTooltip("Plugins are shared libraries that have a class inheriting from PluginApplication and "
                               "define a plugin entry point. Look at Samples/103_GamePlugin for more information.");
        }
        else
        {
            for (const String& baseName : pluginNames)
            {
                PluginManager* plugins = project_->GetPlugins();
                Plugin* plugin = plugins->GetPlugin(baseName);
                bool loaded = plugin != nullptr;
                bool editorOnly = plugin && plugin->GetFlags() & PLUGIN_PRIVATE;

                ui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0);
                if (ui::EditorToolbarButton(ICON_FA_BATTERY_EMPTY, "Inactive", !loaded) && loaded)
                    plugins->Unload(plugin);

                if (ui::EditorToolbarButton(ICON_FA_BATTERY_HALF, "Editor-only", loaded && editorOnly))
                {
                    if (!loaded)
                    {
                        plugins->Load(baseName);
                        plugin = plugins->GetPlugin(baseName);
                    }
                    plugin->SetFlags(plugin->GetFlags() | PLUGIN_PRIVATE);
                }
                if (ui::EditorToolbarButton(ICON_FA_BATTERY_FULL, "Editor and Game", loaded && !editorOnly))
                {
                    if (!loaded)
                    {
                        plugins->Load(baseName);
                        plugin = plugins->GetPlugin(baseName);
                    }
                    plugin->SetFlags(plugin->GetFlags() & ~PLUGIN_PRIVATE);
                }
                ui::PopStyleVar();
                ui::SameLine();
                ui::TextUnformatted(baseName.CString());
            }
        }
        ui::PopID();    // Plugins
        ui::EndMenu();
    }
#endif
    if (ui::BeginMenu("Main Scene"))
    {
        ui::PushID("Main Scene");
        auto* sceneNames = ui::GetUIState<StringVector>();
        if (sceneNames->Empty())
        {
            GetFileSystem()->ScanDir(*sceneNames, project_->GetResourcePath(), "*.xml", SCAN_FILES, true);
            for (auto it = sceneNames->Begin(); it != sceneNames->End();)
            {
                if (GetContentType(*it) == CTYPE_SCENE)
                    ++it;
                else
                    it = sceneNames->Erase(it);
            }
        }

        for (const String& resourceName : *sceneNames)
        {
            bool isDefaultScene = resourceName == project_->GetDefaultSceneName();
            if (ui::Checkbox(resourceName.CString(), &isDefaultScene))
            {
                if (isDefaultScene)
                    project_->SetDefaultSceneName(resourceName);
            }
        }

        if (sceneNames->Empty())
            ui::TextUnformatted("Create a new scene first.");

        ui::PopID();    // Main Scene
        ui::EndMenu();
    }

    if (ui::BeginMenu("Settings"))
    {
        static const VariantType variantTypes[] = {
            VAR_BOOL,
            VAR_INT,
            VAR_INT64,
            VAR_FLOAT,
            VAR_DOUBLE,
            VAR_COLOR,
            VAR_STRING,
        };

        static const char* variantNames[] = {
            "Bool",
            "Int",
            "Int64",
            "Float",
            "Double",
            "Color",
            "String",
        };

        static const char* predefinedNames[] = {
            "Select Option Name",
            "Enter Custom",
            EP_AUTOLOAD_PATHS.CString(),
            EP_BORDERLESS.CString(),
            EP_DUMP_SHADERS.CString(),
            EP_FLUSH_GPU.CString(),
            EP_FORCE_GL2.CString(),
            EP_FRAME_LIMITER.CString(),
            EP_FULL_SCREEN.CString(),
            EP_HEADLESS.CString(),
            EP_HIGH_DPI.CString(),
            EP_LOG_LEVEL.CString(),
            EP_LOG_NAME.CString(),
            EP_LOG_QUIET.CString(),
            EP_LOW_QUALITY_SHADOWS.CString(),
            EP_MATERIAL_QUALITY.CString(),
            EP_MONITOR.CString(),
            EP_MULTI_SAMPLE.CString(),
            EP_ORGANIZATION_NAME.CString(),
            EP_ORIENTATIONS.CString(),
            EP_PACKAGE_CACHE_DIR.CString(),
            EP_RENDER_PATH.CString(),
            EP_REFRESH_RATE.CString(),
            EP_RESOURCE_PACKAGES.CString(),
            EP_RESOURCE_PATHS.CString(),
            EP_RESOURCE_PREFIX_PATHS.CString(),
            EP_SHADER_CACHE_DIR.CString(),
            EP_SHADOWS.CString(),
            EP_SOUND.CString(),
            EP_SOUND_BUFFER.CString(),
            EP_SOUND_INTERPOLATION.CString(),
            EP_SOUND_MIX_RATE.CString(),
            EP_SOUND_STEREO.CString(),
            EP_TEXTURE_ANISOTROPY.CString(),
            EP_TEXTURE_FILTER_MODE.CString(),
            EP_TEXTURE_QUALITY.CString(),
            EP_TOUCH_EMULATION.CString(),
            EP_TRIPLE_BUFFER.CString(),
            EP_VSYNC.CString(),
            EP_WINDOW_HEIGHT.CString(),
            EP_WINDOW_ICON.CString(),
            EP_WINDOW_POSITION_X.CString(),
            EP_WINDOW_POSITION_Y.CString(),
            EP_WINDOW_RESIZABLE.CString(),
            EP_WINDOW_TITLE.CString(),
            EP_WINDOW_WIDTH.CString(),
            EP_WORKER_THREADS.CString(),
        };

        static VariantType predefinedTypes[] = {
            VAR_NONE,   // Select Option Name
            VAR_NONE,   // Enter Custom
            VAR_STRING, // EP_AUTOLOAD_PATHS
            VAR_BOOL,   // EP_BORDERLESS
            VAR_BOOL,   // EP_DUMP_SHADERS
            VAR_BOOL,   // EP_FLUSH_GPU
            VAR_BOOL,   // EP_FORCE_GL2
            VAR_BOOL,   // EP_FRAME_LIMITER
            VAR_BOOL,   // EP_FULL_SCREEN
            VAR_BOOL,   // EP_HEADLESS
            VAR_BOOL,   // EP_HIGH_DPI
            VAR_INT,    // EP_LOG_LEVEL
            VAR_STRING, // EP_LOG_NAME
            VAR_BOOL,   // EP_LOG_QUIET
            VAR_BOOL,   // EP_LOW_QUALITY_SHADOWS
            VAR_INT,    // EP_MATERIAL_QUALITY
            VAR_INT,    // EP_MONITOR
            VAR_INT,    // EP_MULTI_SAMPLE
            VAR_STRING, // EP_ORGANIZATION_NAME
            VAR_STRING, // EP_ORIENTATIONS
            VAR_STRING, // EP_PACKAGE_CACHE_DIR
            VAR_STRING, // EP_RENDER_PATH
            VAR_INT,    // EP_REFRESH_RATE
            VAR_STRING, // EP_RESOURCE_PACKAGES
            VAR_STRING, // EP_RESOURCE_PATHS
            VAR_STRING, // EP_RESOURCE_PREFIX_PATHS
            VAR_STRING, // EP_SHADER_CACHE_DIR
            VAR_BOOL,   // EP_SHADOWS
            VAR_BOOL,   // EP_SOUND
            VAR_INT,    // EP_SOUND_BUFFER
            VAR_BOOL,   // EP_SOUND_INTERPOLATION
            VAR_INT,    // EP_SOUND_MIX_RATE
            VAR_BOOL,   // EP_SOUND_STEREO
            VAR_INT,    // EP_TEXTURE_ANISOTROPY
            VAR_INT,    // EP_TEXTURE_FILTER_MODE
            VAR_INT,    // EP_TEXTURE_QUALITY
            VAR_BOOL,   // EP_TOUCH_EMULATION
            VAR_BOOL,   // EP_TRIPLE_BUFFER
            VAR_BOOL,   // EP_VSYNC
            VAR_INT,    // EP_WINDOW_HEIGHT
            VAR_STRING, // EP_WINDOW_ICON
            VAR_INT,    // EP_WINDOW_POSITION_X
            VAR_INT,    // EP_WINDOW_POSITION_Y
            VAR_BOOL,   // EP_WINDOW_RESIZABLE
            VAR_STRING, // EP_WINDOW_TITLE
            VAR_INT,    // EP_WINDOW_WIDTH
            VAR_INT,    // EP_WORKER_THREADS
        };

        static_assert(URHO3D_ARRAYSIZE(predefinedNames) == URHO3D_ARRAYSIZE(predefinedNames), "Sizes must match.");

        struct NewEntryState
        {
            std::string customName;
            int customType = 0;
            int predefinedItem = 0;
        };

        auto* state = ui::GetUIState<NewEntryState>();
        auto& settings = project_->GetDefaultEngineSettings();
        for (auto it = settings.Begin(); it != settings.End();)
        {
            const String& settingName = it->first_;
            ui::IdScope idScope(settingName.CString());
            Variant& value = it->second_;
            float startPos = ui::GetCursorPosX();
            ui::TextUnformatted(settingName.CString());
            ui::SameLine();
            ui::SetCursorPosX(startPos = startPos + 180_dpx + ui::GetStyle().ItemSpacing.x);
            UI_ITEMWIDTH(100_dpx)
                RenderSingleAttribute(value);
            ui::SameLine();
            ui::SetCursorPosX(startPos + 100_dpx + ui::GetStyle().ItemSpacing.x);
            if (ui::Button(ICON_FA_TRASH))
                it = settings.Erase(it);
            else
                ++it;
        }

        UI_ITEMWIDTH(280_dpx)
            ui::Combo("###Selector", &state->predefinedItem, predefinedNames, URHO3D_ARRAYSIZE(predefinedNames));

        ui::SameLine();

        const char* cantSubmitHelpText = nullptr;
        if (state->predefinedItem == 0)
            cantSubmitHelpText = "Parameter is not selected.";
        else if (state->predefinedItem == 1)
        {
            if (state->customName.empty())
                cantSubmitHelpText = "Custom name can not be empty.";
            else if (settings.Find(state->customName.c_str()) != settings.End())
                cantSubmitHelpText = "Parameter with same name is already added.";
        }
        else if (state->predefinedItem > 1 && settings.Find(predefinedNames[state->predefinedItem]) != settings.End())
            cantSubmitHelpText = "Parameter with same name is already added.";

        ui::PushStyleColor(ImGuiCol_Button, ui::GetStyle().Colors[cantSubmitHelpText == nullptr ? ImGuiCol_Button : ImGuiCol_TextDisabled]);
        if (ui::Button(ICON_FA_CHECK) && cantSubmitHelpText == nullptr)
        {
            if (state->predefinedItem == 1)
                settings.Insert({state->customName.c_str(), Variant{variantTypes[state->customType]}});
            else
                settings.Insert({predefinedNames[state->predefinedItem], Variant{predefinedTypes[state->predefinedItem]}});
            state->customName.clear();
            state->customType = 0;
        }
        ui::PopStyleColor();
        if (cantSubmitHelpText)
            ui::SetHelpTooltip(cantSubmitHelpText, KEY_UNKNOWN);

        if (state->predefinedItem == 1)
        {
            UI_ITEMWIDTH(180_dpx)
                ui::InputText("###Key", &state->customName);

            // Custom entry type selector
            ui::SameLine();
            UI_ITEMWIDTH(100_dpx)
                ui::Combo("###Type", &state->customType, variantNames, SDL_arraysize(variantTypes));
        }
        ui::EndMenu();
    }

    ui::Separator();

    if (ui::MenuItem(ICON_FA_BOXES " Package files"))
    {
        GetSubsystem<Project>()->GetPipeline().CreatePaksAsync();
    }
}

}
