//
// Copyright (c) 2008-2018 the Urho3D project.
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

#include "../Precompiled.h"

#include "../Audio/Audio.h"
#include "../Core/Context.h"
#include "../Core/CoreEvents.h"
#include "../Core/Profiler.h"
#include "../Core/ProcessUtils.h"
#include "../Core/Thread.h"
#include "../Core/WorkQueue.h"
#ifdef URHO3D_SYSTEMUI
#include "../SystemUI/SystemUI.h"
#include "../SystemUI/Console.h"
#include "../SystemUI/DebugHud.h"
#endif
#include "../Engine/Engine.h"
#include "../Engine/EngineDefs.h"
#include "../Graphics/Graphics.h"
#include "../Graphics/Renderer.h"
#include "../Input/Input.h"
#include "../IO/FileSystem.h"
#include "../IO/Log.h"
#include "../IO/PackageFile.h"
#include "IO/FileWatcher.h"
#include "../Misc/FreeFunctions.h"
#ifdef URHO3D_IK
#include "../IK/IK.h"
#endif
#ifdef URHO3D_NAVIGATION
#include "../Navigation/NavigationMesh.h"
#endif
#ifdef URHO3D_NETWORK
#include "../Network/Network.h"
#endif
#ifdef URHO3D_PHYSICS
#include "Physics/PhysicsWorld.h"
#endif
#include "../Resource/ResourceCache.h"
#include "../Resource/Localization.h"
#include "../Scene/Scene.h"
#include "../Scene/SceneEvents.h"
#include "../UI/UI.h"
#ifdef URHO3D_URHO2D
#include "../Urho2D/Urho2D.h"
#endif
#if URHO3D_TASKS
#include "../Core/Tasks.h"
#endif
#include "../Engine/EngineEvents.h"

#if defined(__EMSCRIPTEN__) && defined(URHO3D_TESTING)
#include <emscripten/emscripten.h>
#endif

#include <CLI11/CLI11.hpp>

#include "../DebugNew.h"



#if defined(_MSC_VER) && defined(_DEBUG)
// From dbgint.h
#define nNoMansLandSize 4

typedef struct _CrtMemBlockHeader
{
    struct _CrtMemBlockHeader* pBlockHeaderNext;
    struct _CrtMemBlockHeader* pBlockHeaderPrev;
    char* szFileName;
    int nLine;
    size_t nDataSize;
    int nBlockUse;
    long lRequest;
    unsigned char gap[nNoMansLandSize];
} _CrtMemBlockHeader;
#endif

namespace Urho3D
{

extern const char* logLevelPrefixes[];

Engine::Engine(Context* context) :
    Object(context),
#if defined(IOS) || defined(TVOS) || defined(__ANDROID__) || defined(__arm__) || defined(__aarch64__)
    pauseMinimized_(true),
#else
    pauseMinimized_(false),
#endif
#ifdef URHO3D_TESTING
    timeOut_(0),
#endif
    autoExit_(true),
    initialized_(false),
    exiting_(false),
    headless_(false),
    audioPaused_(false)
{



    // Register self as a subsystem
    context_->RegisterSubsystem(this);

    // Create subsystems which do not depend on engine initialization or startup parameters
    context_->RegisterSubsystem(new Time(context_));
    context_->RegisterSubsystem(new WorkQueue(context_));
    context_->RegisterSubsystem(new FileSystem(context_));
	context_->RegisterFactory<FileWatcher>();
#ifdef URHO3D_LOGGING
    context_->RegisterSubsystem(new Log(context_));
#endif
    context_->RegisterSubsystem(new ResourceCache(context_));
    context_->RegisterSubsystem(new Localization(context_));
#ifdef URHO3D_NETWORK
    context_->RegisterSubsystem(new Network(context_));
#endif
    context_->RegisterSubsystem(new Input(context_));
    context_->RegisterSubsystem(new Audio(context_));
    context_->RegisterSubsystem(new UI(context_));
#if URHO3D_TASKS
    context_->RegisterSubsystem(new Tasks(context_));
#endif
	context_->RegisterSubsystem(new FreeFunctions(context_));
    // Register object factories for libraries which are not automatically registered along with subsystem creation
    RegisterUILibrary(context_);

    RegisterSceneLibrary(context_);

#ifdef URHO3D_TASKS
    context_->RegisterFactory<Task>();
#endif

#ifdef URHO3D_IK
    RegisterIKLibrary(context_);
#endif

#ifdef URHO3D_PHYSICS
    RegisterPhysicsLibrary(context_);
#endif

#ifdef URHO3D_NAVIGATION
    RegisterNavigationLibrary(context_);
#endif




    SubscribeToEvent(E_EXITREQUESTED, URHO3D_HANDLER(Engine, HandleExitRequested));
}

Engine::~Engine() = default;

bool Engine::Initialize(const VariantMap& parameters)
{
    if (initialized_)
        return true;

    URHO3D_PROFILE_FUNCTION();

    // Set headless mode
    headless_ = GetParameter(parameters, EP_HEADLESS, false).GetBool();

    // Register the rest of the subsystems
    if (!headless_)
    {
        context_->RegisterSubsystem(new Graphics(context_));
        context_->RegisterSubsystem(new Renderer(context_));
        context_->graphics_ = context_->GetSubsystem<Graphics>();
        context_->renderer_ = context_->GetSubsystem<Renderer>();
    }
    else
    {
        // Register graphics library objects explicitly in headless mode to allow them to work without using actual GPU resources
        RegisterGraphicsLibrary(context_);
    }

#ifdef URHO3D_URHO2D
    // 2D graphics library is dependent on 3D graphics library
    RegisterUrho2DLibrary(context_);
#endif

    // Start logging
    auto* log = GetSubsystem<Log>();
    if (log)
    {
        if (HasParameter(parameters, EP_LOG_LEVEL))
            log->SetLevel(static_cast<LogLevel>(GetParameter(parameters, EP_LOG_LEVEL).GetInt()));
        log->SetQuiet(GetParameter(parameters, EP_LOG_QUIET, false).GetBool());
        log->Open(GetParameter(parameters, EP_LOG_NAME, "Urho3D.log").GetString());
    }

    // Set maximally accurate low res timer
    GetSubsystem<Time>()->SetTimerPeriod(1);

    // Configure max FPS
    //if (GetParameter(parameters, EP_FRAME_LIMITER, true) == false)
    //    SetMaxFps(0);

    // Set amount of worker threads according to the available physical CPU cores. Using also hyperthreaded cores results in
    // unpredictable extra synchronization overhead. Also reserve one core for the main thread
#ifdef URHO3D_THREADING
    unsigned numThreads = GetParameter(parameters, EP_WORKER_THREADS, true).GetBool() ? GetNumPhysicalCPUs() - 1 : 0;
    if (numThreads)
    {
        GetSubsystem<WorkQueue>()->CreateThreads(numThreads);

        URHO3D_LOGINFOF("Created %u worker thread%s", numThreads, numThreads > 1 ? "s" : "");
    }
#endif

    // Add resource paths
    if (!InitializeResourceCache(parameters, false))
        return false;

    auto* cache = GetSubsystem<ResourceCache>();
    auto* fileSystem = GetSubsystem<FileSystem>();

    // Initialize graphics & audio output
    if (!headless_)
    {
        auto* graphics = GetSubsystem<Graphics>();
        auto* renderer = GetSubsystem<Renderer>();

        if (HasParameter(parameters, EP_EXTERNAL_WINDOW))
            graphics->SetExternalWindow(GetParameter(parameters, EP_EXTERNAL_WINDOW).GetVoidPtr());
        graphics->SetWindowTitle(GetParameter(parameters, EP_WINDOW_TITLE, "Urho3D").GetString());
        graphics->SetWindowIcon(cache->GetResource<Image>(GetParameter(parameters, EP_WINDOW_ICON, String::EMPTY).GetString()));
        graphics->SetFlushGPU(GetParameter(parameters, EP_FLUSH_GPU, false).GetBool());
        graphics->SetOrientations(GetParameter(parameters, EP_ORIENTATIONS, "LandscapeLeft LandscapeRight").GetString());

        if (HasParameter(parameters, EP_WINDOW_POSITION_X) && HasParameter(parameters, EP_WINDOW_POSITION_Y))
            graphics->SetWindowPosition(GetParameter(parameters, EP_WINDOW_POSITION_X).GetInt(),
                GetParameter(parameters, EP_WINDOW_POSITION_Y).GetInt());

#ifdef URHO3D_OPENGL
        if (HasParameter(parameters, EP_FORCE_GL2))
            graphics->SetForceGL2(GetParameter(parameters, EP_FORCE_GL2).GetBool());
#endif

        if (!graphics->SetMode(
            GetParameter(parameters, EP_WINDOW_WIDTH, 0).GetInt(),
            GetParameter(parameters, EP_WINDOW_HEIGHT, 0).GetInt(),
            GetParameter(parameters, EP_FULL_SCREEN, true).GetBool(),
            GetParameter(parameters, EP_BORDERLESS, false).GetBool(),
            GetParameter(parameters, EP_WINDOW_RESIZABLE, false).GetBool(),
            GetParameter(parameters, EP_HIGH_DPI, true).GetBool(),
            GetParameter(parameters, EP_VSYNC, false).GetBool(),
            GetParameter(parameters, EP_TRIPLE_BUFFER, false).GetBool(),
            GetParameter(parameters, EP_MULTI_SAMPLE, 1).GetInt(),
            GetParameter(parameters, EP_MONITOR, 0).GetInt(),
            GetParameter(parameters, EP_REFRESH_RATE, 0).GetInt()
        ))
            return false;

        graphics->SetShaderCacheDir(GetParameter(parameters, EP_SHADER_CACHE_DIR, fileSystem->GetAppPreferencesDir("urho3d", "shadercache")).GetString());

        if (HasParameter(parameters, EP_DUMP_SHADERS))
            graphics->BeginDumpShaders(GetParameter(parameters, EP_DUMP_SHADERS, String::EMPTY).GetString());
        if (HasParameter(parameters, EP_RENDER_PATH))
            renderer->SetDefaultRenderPath(cache->GetResource<XMLFile>(GetParameter(parameters, EP_RENDER_PATH).GetString()));

        renderer->SetDrawShadows(GetParameter(parameters, EP_SHADOWS, true).GetBool());
        if (renderer->GetDrawShadows() && GetParameter(parameters, EP_LOW_QUALITY_SHADOWS, false).GetBool())
            renderer->SetShadowQuality(SHADOWQUALITY_SIMPLE_16BIT);
        renderer->SetMaterialQuality((MaterialQuality)GetParameter(parameters, EP_MATERIAL_QUALITY, QUALITY_HIGH).GetInt());
        renderer->SetTextureQuality((MaterialQuality)GetParameter(parameters, EP_TEXTURE_QUALITY, QUALITY_HIGH).GetInt());
        renderer->SetTextureFilterMode((TextureFilterMode)GetParameter(parameters, EP_TEXTURE_FILTER_MODE, FILTER_TRILINEAR).GetInt());
        renderer->SetTextureAnisotropy(GetParameter(parameters, EP_TEXTURE_ANISOTROPY, 4).GetInt());

        if (GetParameter(parameters, EP_SOUND, true).GetBool())
        {
            GetSubsystem<Audio>()->SetMode(
                GetParameter(parameters, EP_SOUND_BUFFER, 100).GetInt(),
                GetParameter(parameters, EP_SOUND_MIX_RATE, 44100).GetInt(),
                GetParameter(parameters, EP_SOUND_STEREO, true).GetBool(),
                GetParameter(parameters, EP_SOUND_INTERPOLATION, true).GetBool()
            );
        }
    }

    // Init FPU state of main thread
    InitFPU();

    // Initialize input
    if (HasParameter(parameters, EP_TOUCH_EMULATION))
        GetSubsystem<Input>()->SetTouchEmulation(GetParameter(parameters, EP_TOUCH_EMULATION).GetBool());

    // Initialize network
#ifdef URHO3D_NETWORK
    if (HasParameter(parameters, EP_PACKAGE_CACHE_DIR))
        GetSubsystem<Network>()->SetPackageCacheDir(GetParameter(parameters, EP_PACKAGE_CACHE_DIR).GetString());
#endif

#ifdef URHO3D_TESTING
    if (HasParameter(parameters, EP_TIME_OUT))
        timeOut_ = GetParameter(parameters, EP_TIME_OUT, 0).GetInt() * 1000000LL;
#endif
    if (!headless_)
    {
#ifdef URHO3D_SYSTEMUI
        context_->RegisterSubsystem(new SystemUI(context_));
#endif
    }




    updateTimerTracker_.Reset();
	renderTimerTracker_.Reset();

	updateUpdateTimeTimer();
	updateRenderTimeTimer();


    URHO3D_LOGINFO("Initialized engine");
    initialized_ = true;
    SendEvent(E_ENGINEINITIALIZED);
    return true;
}

bool Engine::InitializeResourceCache(const VariantMap& parameters, bool removeOld /*= true*/)
{
    auto* cache = GetSubsystem<ResourceCache>();
    auto* fileSystem = GetSubsystem<FileSystem>();

    // Remove all resource paths and packages
    if (removeOld)
    {
        Vector<String> resourceDirs = cache->GetResourceDirs();
        Vector<SharedPtr<PackageFile> > packageFiles = cache->GetPackageFiles();
        for (unsigned i = 0; i < resourceDirs.Size(); ++i)
            cache->RemoveResourceDir(resourceDirs[i]);
        for (unsigned i = 0; i < packageFiles.Size(); ++i)
            cache->RemovePackageFile(packageFiles[i]);
    }

    // Add resource paths
    Vector<String> resourcePrefixPaths = GetParameter(parameters, EP_RESOURCE_PREFIX_PATHS, String::EMPTY).GetString().Split(';', true);
    for (unsigned i = 0; i < resourcePrefixPaths.Size(); ++i)
        resourcePrefixPaths[i] = AddTrailingSlash(
            IsAbsolutePath(resourcePrefixPaths[i]) ? resourcePrefixPaths[i] : fileSystem->GetProgramDir() + resourcePrefixPaths[i]);
    Vector<String> resourcePaths = GetParameter(parameters, EP_RESOURCE_PATHS, "Data;CoreData").GetString().Split(';');
    Vector<String> resourcePackages = GetParameter(parameters, EP_RESOURCE_PACKAGES).GetString().Split(';');
    Vector<String> autoLoadPaths = GetParameter(parameters, EP_AUTOLOAD_PATHS, "Autoload").GetString().Split(';');

    for (unsigned i = 0; i < resourcePaths.Size(); ++i)
    {
        // If path is not absolute, prefer to add it as a package if possible
        if (!IsAbsolutePath(resourcePaths[i]))
        {
            unsigned j = 0;
            for (; j < resourcePrefixPaths.Size(); ++j)
            {
                String packageName = resourcePrefixPaths[j] + resourcePaths[i] + ".pak";
                if (fileSystem->FileExists(packageName))
                {
                    if (cache->AddPackageFile(packageName))
                        break;
                    else
                        return false;   // The root cause of the error should have already been logged
                }
                String pathName = resourcePrefixPaths[j] + resourcePaths[i];
                if (fileSystem->DirExists(pathName))
                {
                    if (cache->AddResourceDir(pathName))
                        break;
                    else
                        return false;
                }
            }
            if (j == resourcePrefixPaths.Size() && !headless_)
            {
                URHO3D_LOGERRORF(
                    "Failed to add resource path '%s', check the documentation on how to set the 'resource prefix path'",
                    resourcePaths[i].CString());
                return false;
            }
        }
        else
        {
            String pathName = resourcePaths[i];
            if (fileSystem->DirExists(pathName))
                if (!cache->AddResourceDir(pathName))
                    return false;
        }
    }

    // Then add specified packages
    for (unsigned i = 0; i < resourcePackages.Size(); ++i)
    {
        unsigned j = 0;
        for (; j < resourcePrefixPaths.Size(); ++j)
        {
            String packageName = resourcePrefixPaths[j] + resourcePackages[i];
            if (fileSystem->FileExists(packageName))
            {
                if (cache->AddPackageFile(packageName))
                    break;
                else
                    return false;
            }
        }
        if (j == resourcePrefixPaths.Size() && !headless_)
        {
            URHO3D_LOGERRORF(
                "Failed to add resource package '%s', check the documentation on how to set the 'resource prefix path'",
                resourcePackages[i].CString());
            return false;
        }
    }

    // Add auto load folders. Prioritize these (if exist) before the default folders
    for (unsigned i = 0; i < autoLoadPaths.Size(); ++i)
    {
        bool autoLoadPathExist = false;

        for (unsigned j = 0; j < resourcePrefixPaths.Size(); ++j)
        {
            String autoLoadPath(autoLoadPaths[i]);
            if (!IsAbsolutePath(autoLoadPath))
                autoLoadPath = resourcePrefixPaths[j] + autoLoadPath;

            if (fileSystem->DirExists(autoLoadPath))
            {
                autoLoadPathExist = true;

                // Add all the subdirs (non-recursive) as resource directory
                Vector<String> subdirs;
                fileSystem->ScanDir(subdirs, autoLoadPath, "*", SCAN_DIRS, false);
                for (unsigned y = 0; y < subdirs.Size(); ++y)
                {
                    String dir = subdirs[y];
                    if (dir.StartsWith("."))
                        continue;

                    String autoResourceDir = AddTrailingSlash(autoLoadPath) + dir;
                    if (!cache->AddResourceDir(autoResourceDir, 0))
                        return false;
                }

                // Add all the found package files (non-recursive)
                Vector<String> paks;
                fileSystem->ScanDir(paks, autoLoadPath, "*.pak", SCAN_FILES, false);
                for (unsigned y = 0; y < paks.Size(); ++y)
                {
                    String pak = paks[y];
                    if (pak.StartsWith("."))
                        continue;

                    String autoPackageName = autoLoadPath + "/" + pak;
                    if (!cache->AddPackageFile(autoPackageName, 0))
                        return false;
                }
            }
        }

        // The following debug message is confusing when user is not aware of the autoload feature
        // Especially because the autoload feature is enabled by default without user intervention
        // The following extra conditional check below is to suppress unnecessary debug log entry under such default situation
        // The cleaner approach is to not enable the autoload by default, i.e. do not use 'Autoload' as default value for 'AutoloadPaths' engine parameter
        // However, doing so will break the existing applications that rely on this
        if (!autoLoadPathExist && (autoLoadPaths.Size() > 1 || autoLoadPaths[0] != "Autoload"))
            URHO3D_LOGDEBUGF(
                "Skipped autoload path '%s' as it does not exist, check the documentation on how to set the 'resource prefix path'",
                autoLoadPaths[i].CString());
    }

    return true;
}

Console* Engine::CreateConsole()
{
    if (headless_ || !initialized_)
        return nullptr;

#ifdef URHO3D_SYSTEMUI
    // Return existing console if possible
    auto* console = GetSubsystem<Console>();
    if (!console)
    {
        console = new Console(context_);
        context_->RegisterSubsystem(console);
    }

    return console;
#else
    return nullptr;
#endif
}

DebugHud* Engine::CreateDebugHud()
{
    if (headless_ || !initialized_)
        return nullptr;

#ifdef URHO3D_SYSTEMUI
    // Return existing debug HUD if possible
    auto* debugHud = GetSubsystem<DebugHud>();
    if (!debugHud)
    {
        debugHud = new DebugHud(context_);
        context_->RegisterSubsystem(debugHud);
    }

    return debugHud;
#else
    return nullptr;
#endif
}





void Engine::SetRenderTimeGoalUs(unsigned timeUs)
{
	renderTimeGoalUs_ = timeUs;
	updateRenderTimeTimer();
}




void Engine::SetUpdateTimeGoalUs(unsigned timeUs)
{
	updateTimeGoalUs_ = timeUs/8;
	updateUpdateTimeTimer();
}


void Engine::SetPauseMinimized(bool enable)
{
    pauseMinimized_ = enable;
}

void Engine::SetAutoExit(bool enable)
{
    // On mobile platforms exit is mandatory if requested by the platform itself and should not be attempted to be disabled
#if defined(__ANDROID__) || defined(IOS) || defined(TVOS)
    enable = true;
#endif
    autoExit_ = enable;
}


void Engine::Exit()
{
#if defined(IOS) || defined(TVOS)
    // On iOS/tvOS it's not legal for the application to exit on its own, instead it will be minimized with the home key
#else
    DoExit();
#endif
}

void Engine::DumpProfiler()
{
#ifdef URHO3D_LOGGING
    if (!Thread::IsMainThread())
        return;

    // TODO: Put something here or remove API
#endif
}

void Engine::DumpResources(bool dumpFileName)
{
#ifdef URHO3D_LOGGING
    if (!Thread::IsMainThread())
        return;

    auto* cache = GetSubsystem<ResourceCache>();
    const HashMap<StringHash, ResourceGroup>& resourceGroups = cache->GetAllResources();
    if (dumpFileName)
    {
        URHO3D_LOGRAW("Used resources:\n");
        for (HashMap<StringHash, ResourceGroup>::ConstIterator i = resourceGroups.Begin(); i != resourceGroups.End(); ++i)
        {
            const HashMap<StringHash, SharedPtr<Resource> >& resources = i->second_.resources_;
            if (dumpFileName)
            {
                for (HashMap<StringHash, SharedPtr<Resource> >::ConstIterator j = resources.Begin(); j != resources.End(); ++j)
                    URHO3D_LOGRAW(j->second_->GetName() + "\n");
            }
        }
    }
    else
        URHO3D_LOGRAW(cache->PrintMemoryUsage() + "\n");
#endif
}

void Engine::DumpMemory()
{
#ifdef URHO3D_LOGGING
#if defined(_MSC_VER) && defined(_DEBUG)
    _CrtMemState state;
    _CrtMemCheckpoint(&state);
    _CrtMemBlockHeader* block = state.pBlockHeader;
    unsigned total = 0;
    unsigned blocks = 0;

    for (;;)
    {
        if (block && block->pBlockHeaderNext)
            block = block->pBlockHeaderNext;
        else
            break;
    }

    while (block)
    {
        if (block->nBlockUse > 0)
        {
            if (block->szFileName)
                URHO3D_LOGRAW("Block " + String((int)block->lRequest) + ": " + String(block->nDataSize) + " bytes, file " + String(block->szFileName) + " line " + String(block->nLine) + "\n");
            else
                URHO3D_LOGRAW("Block " + String((int)block->lRequest) + ": " + String(block->nDataSize) + " bytes\n");

            total += block->nDataSize;
            ++blocks;
        }
        block = block->pBlockHeaderPrev;
    }

    URHO3D_LOGRAW("Total allocated memory " + String(total) + " bytes in " + String(blocks) + " blocks\n\n");
#else
    URHO3D_LOGRAW("DumpMemory() supported on MSVC debug mode only\n\n");
#endif
#endif
}

unsigned Engine::FreeUpdate()
{
    URHO3D_PROFILE_FUNCTION();
	// If not headless, and the graphics subsystem no longer has a window open, assume we should exit
	if (!headless_ && !GetSubsystem<Graphics>()->IsInitialized())
		exiting_ = true;

	if (exiting_)
		return 0;


	// Note: there is a minimal performance cost to looking up subsystems (uses a hashmap); if they would be looked up several
	// times per frame it would be better to cache the pointers
	auto* time = static_cast<Time*>(context_->time_);
	auto* input = static_cast<Input*>(context_->input_);

	updateAudioPausing();


    //if we have not rendered in a long time - we are overloaded so just render once every second to indicate the program is still alive.
    if (renderTimer_.GetUSec(false) > 100000)
    {
        renderTimer_.Reset();
        Render();

    }
	else if (updateTimer_.IsTimedOut()) {
		updateTimer_.Reset();
        URHO3D_PROFILE_FRAME();//sync profiling frames on the start of updates.
		Update();
	}
	else if (renderTimer_.IsTimedOut())
	{
		//Render
		renderTimer_.Reset();
		Render();
	}


    {

        //lets compute approximate time we have until next update or render
        long long updateTimeLeft = (updateTimer_.GetTimeoutDuration() - updateTimer_.GetUSec(false));
        long long renderTimeLeft = (renderTimer_.GetTimeoutDuration() - renderTimer_.GetUSec(false));

        long long timeLeftUS = Urho3D::Min(updateTimeLeft, renderTimeLeft);
        if (timeLeftUS > 0)
            return unsigned(timeLeftUS);
        else
            return 0;
    }


	return 0;
}



void Engine::Update()
{
    URHO3D_PROFILE_FUNCTION();

	//compute times
	updateTick_++;
	lastUpdateTimeUs_ = updateTimerTracker_.GetUSec(true);



    SendUpdateEvents();

}

void Engine::SendUpdateEvents()
{


        VariantMap& eventData = GetEventDataMap();




        eventData[Update::P_TIMESTEP] = (float(lastUpdateTimeUs_) / 1000000.0f);
        eventData[Update::P_TARGET_TIMESTEP] = (float(updateTimeGoalUs_) / 1000000.0f);
        eventData[Update::P_UPDATETICK] = updateTick_;


        SendEvent(E_PREUPDATE, eventData);

        SendEvent(E_UPDATE, eventData);
        // Logic post-update event
        SendEvent(E_POSTUPDATE, eventData);

        SendEvent(E_ENDFRAME);

        SendEvent(E_ENDFRAMEFINAL);

}

void Engine::Render()
{
    if (headless_)
        return;

    URHO3D_PROFILE_FUNCTION();

    // If device is lost, BeginFrame will fail and we skip rendering
    auto* graphics = GetSubsystem<Graphics>();
    if (!graphics->BeginFrame())
        return;

	//compute times
	renderTick_++;
	lastRenderTimeUs_ = renderTimerTracker_.GetUSec(true);



	VariantMap& eventData = GetEventDataMap();
	eventData[RenderUpdate::P_TIMESTEP] = float(lastRenderTimeUs_)/1000.0f;
	eventData[RenderUpdate::P_RENDERTICK] = renderTick_;
	
	// Rendering update event
	SendEvent(E_RENDERUPDATE, eventData);


	// Post-render update event
	SendEvent(E_POSTRENDERUPDATE, eventData);


    GetSubsystem<Renderer>()->Render();
    GetSubsystem<UI>()->Render();
    graphics->ResetRenderTargets();
    graphics->EndFrame();



}

void Engine::DefineParameters(CLI::App& commandLine, VariantMap& engineParameters)
{
    auto addFlagInternal = [&](const char* name, const char* description, CLI::callback_t fun) {
        CLI::Option *opt = commandLine.add_option(name, fun, description, false);
        if(opt->get_positional())
            throw CLI::IncorrectConstruction::PositionalFlag(name);
        opt->set_custom_option("", 0);
        return opt;
    };

    auto addFlag = [&](const char* name, const String& param, bool value, const char* description) {
        CLI::callback_t fun = [&](CLI::results_t) {
            engineParameters[param] = value;
            return true;
        };
        return addFlagInternal(name, description, fun);
    };

    auto addOptionPrependString = [&](const char* name, const String& param, const String& value, const char* description) {
        CLI::callback_t fun = [&](CLI::results_t) {
            engineParameters[param] = value + engineParameters[param].GetString();
            return true;
        };
        return addFlagInternal(name, description, fun);
    };

    auto addOptionSetString = [&](const char* name, const String& param, const String& value, const char* description) {
        CLI::callback_t fun = [&](CLI::results_t) {
            engineParameters[param] = value;
            return true;
        };
        return addFlagInternal(name, description, fun);
    };

    auto addOptionString = [&](const char* name, const String& param, const char* description) {
        CLI::callback_t fun = [&](CLI::results_t res) {
            engineParameters[param] = res[0].c_str();
            return true;
        };
        auto* opt = addFlagInternal(name, description, fun);
        opt->set_custom_option("string");
        return opt;
    };

    auto addOptionInt = [&](const char* name, const String& param, const char* description) {
        CLI::callback_t fun = [&](CLI::results_t res) {
            int value = 0;
            if (CLI::detail::lexical_cast(res[0], value))
            {
                engineParameters[param] = value;
                return true;
            }
            return false;
        };
        auto* opt = addFlagInternal(name, description, fun);
        opt->set_custom_option("int");
        return opt;
    };

    auto createOptions = [](const char* format, const char* options[]) {
        StringVector items;
        for (unsigned i = 0; options[i]; i++)
            items.Push(options[i]);
        return ToString(format, String::Joined(items, "|").ToLower().Replaced('_', '-').CString());
    };

    addFlag("--headless", EP_HEADLESS, true, "Do not initialize graphics subsystem");
    addFlag("--nolimit", EP_FRAME_LIMITER, false, "Disable frame limiter");
    addFlag("--flushgpu", EP_FLUSH_GPU, true, "Enable GPU flushing");
    addFlag("--gl2", EP_FORCE_GL2, true, "Force OpenGL2");
    addOptionPrependString("--landscape", EP_ORIENTATIONS, "LandscapeLeft LandscapeRight ", "Force landscape orientation");
    addOptionPrependString("--portrait", EP_ORIENTATIONS, "Portrait PortraitUpsideDown ", "Force portrait orientation");
    addFlag("--nosound", EP_SOUND, false, "Disable sound");
    addFlag("--noip", EP_SOUND_INTERPOLATION, false, "Disable sound interpolation");
    addFlag("--mono", EP_SOUND_STEREO, false, "Force mono sound output (default is stereo)");
    auto* optRenderpath = addOptionString("--renderpath", EP_RENDER_PATH, "Use custom renderpath");
    auto* optPrepass = addOptionSetString("--prepass", EP_RENDER_PATH, "RenderPaths/Prepass.xml", "Use prepass renderpath")->excludes(optRenderpath);
    auto* optDeferred = addOptionSetString("--deferred", EP_RENDER_PATH, "RenderPaths/Deferred.xml", "Use deferred renderpath")->excludes(optRenderpath);
    optRenderpath->excludes(optPrepass)->excludes(optDeferred);
    auto* optNoShadows = addFlag("--noshadows", EP_SHADOWS, false, "Disable shadows");
    auto optLowQualityShadows = addFlag("--lqshadows", EP_LOW_QUALITY_SHADOWS, true, "Use low quality shadows")->excludes(optNoShadows);
    optNoShadows->excludes(optLowQualityShadows);
    addFlag("--nothreads", EP_WORKER_THREADS, false, "Disable multithreading");
    addFlag("-v,--vsync", EP_VSYNC, true, "Enable vsync");
    addFlag("-t,--tripple-buffer", EP_TRIPLE_BUFFER, true, "Enable tripple-buffering");
    addFlag("-w,--windoed", EP_FULL_SCREEN, false, "Windowed mode");
    addFlag("-f,--full-screen", EP_FULL_SCREEN, true, "Full screen mode");
    addFlag("--borderless", EP_BORDERLESS, true, "Borderless window mode");
    addFlag("--lowdpi", EP_HIGH_DPI, false, "Disable high-dpi handling");
    addFlag("--highdpi", EP_HIGH_DPI, true, "Enable high-dpi handling");
    addFlag("-s,--resizeable", EP_WINDOW_RESIZABLE, true, "Enable window resizing");
    addFlag("-q,--quiet", EP_LOG_QUIET, true, "Disable logging");
    addFlagInternal("-l,--log", "Logging level", [&](CLI::results_t res) {
        unsigned logLevel = GetStringListIndex(String(res[0].c_str()).ToUpper().CString(), logLevelPrefixes, M_MAX_UNSIGNED);
        if (logLevel == M_MAX_UNSIGNED)
            return false;
        engineParameters[EP_LOG_LEVEL] = logLevel;
        return true;
    })->set_custom_option(createOptions("string in {%s}", logLevelPrefixes).CString());
    addOptionInt("-x,--height", EP_WINDOW_WIDTH, "Window width");
    addOptionInt("-y,--width", EP_WINDOW_WIDTH, "Window height");
    addOptionInt("--monitor", EP_MONITOR, "Create window on the specified monitor");
    addOptionInt("--hz", EP_REFRESH_RATE, "Use custom refresh rate");
    addOptionInt("-m,--multisample", EP_MULTI_SAMPLE, "Multisampling samples");
    addOptionInt("-b,--sound-buffer", EP_SOUND_BUFFER, "Sound buffer size");
    addOptionInt("-r,--mix-rate", EP_SOUND_MIX_RATE, "Sound mixing rate");
    addOptionString("--pp,--prefix-paths", EP_RESOURCE_PREFIX_PATHS, "Resource prefix paths")->envname("URHO3D_PREFIX_PATH")->set_custom_option("path1;path2;...");
    addOptionString("--pr,--resource-paths", EP_RESOURCE_PATHS, "Resource paths")->set_custom_option("path1;path2;...");
    addOptionString("--pf,--resource-packages", EP_RESOURCE_PACKAGES, "Resource packages")->set_custom_option("path1;path2;...");
    addOptionString("--ap,--autoload-paths", EP_AUTOLOAD_PATHS, "Resource autoload paths")->set_custom_option("path1;path2;...");
    addOptionString("--ds,--dump-shaders", EP_DUMP_SHADERS, "Dump shaders")->set_custom_option("filename");
    addFlagInternal("--mq,--material-quality", "Material quality", [&](CLI::results_t res) {
        unsigned value = 0;
        if (CLI::detail::lexical_cast(res[0], value) && value >= QUALITY_LOW && value <= QUALITY_MAX)
        {
            engineParameters[EP_MATERIAL_QUALITY] = value;
            return true;
        }
        return false;
    })->set_custom_option(ToString("int {%d-%d}", QUALITY_LOW, QUALITY_MAX).CString());
    addFlagInternal("--tq", "Texture quality", [&](CLI::results_t res) {
        unsigned value = 0;
        if (CLI::detail::lexical_cast(res[0], value) && value >= QUALITY_LOW && value <= QUALITY_MAX)
        {
            engineParameters[EP_TEXTURE_QUALITY] = value;
            return true;
        }
        return false;
    })->set_custom_option(ToString("int {%d-%d}", QUALITY_LOW, QUALITY_MAX).CString());
    addFlagInternal("--tf", "Texture filter mode", [&](CLI::results_t res) {
        unsigned mode = GetStringListIndex(String(res[0].c_str()).ToUpper().Replaced('-', '_').CString(), textureFilterModeNames, M_MAX_UNSIGNED);
        if (mode == M_MAX_UNSIGNED)
            return false;
        engineParameters[EP_TEXTURE_FILTER_MODE] = mode;
        return true;
    })->set_custom_option(createOptions("string in {%s}", textureFilterModeNames).CString());
    addFlagInternal("--af", "Use anisotropic filtering", [&](CLI::results_t res) {
        int value = 0;
        if (CLI::detail::lexical_cast(res[0], value) && value >= 1)
        {
            engineParameters[EP_TEXTURE_FILTER_MODE] = FILTER_ANISOTROPIC;
            engineParameters[EP_TEXTURE_ANISOTROPY] = value;
            return true;
        }
        return false;
    })->set_custom_option("int");
    addFlag("--touch", EP_TOUCH_EMULATION, true, "Enable touch emulation");
#ifdef URHO3D_TESTING
    addOptionInt("--timeout", EP_TIME_OUT, "Quit application after specified time");
#endif
}

bool Engine::HasParameter(const VariantMap& parameters, const String& parameter)
{
    StringHash nameHash(parameter);
    return parameters.Find(nameHash) != parameters.End();
}

const Variant& Engine::GetParameter(const VariantMap& parameters, const String& parameter, const Variant& defaultValue)
{
    StringHash nameHash(parameter);
    VariantMap::ConstIterator i = parameters.Find(nameHash);
    return i != parameters.End() ? i->second_ : defaultValue;
}

void Engine::HandleExitRequested(StringHash eventType, VariantMap& eventData)
{
    if (autoExit_)
    {
        // Do not call Exit() here, as it contains mobile platform -specific tests to not exit.
        // If we do receive an exit request from the system on those platforms, we must comply
        DoExit();
    }
}

void Engine::DoExit()
{
    auto* graphics = GetSubsystem<Graphics>();
    if (graphics)
        graphics->Close();

    exiting_ = true;
#if defined(__EMSCRIPTEN__) && defined(URHO3D_TESTING)
    emscripten_force_exit(EXIT_SUCCESS);    // Some how this is required to signal emrun to stop
#endif
}

void Engine::updateAudioPausing()
{
	auto* audio = static_cast<Audio*>(context_->audio_);
	auto* input = GetSubsystem<Input>();

	// If pause when minimized -mode is in use, stop updates and audio as necessary
	if (pauseMinimized_ && input->IsMinimized())
	{
		if (audio->IsPlaying())
		{
			audio->Stop();
			audioPaused_ = true;
		}
	}
	else
	{
		// Only unpause when it was paused by the engine
		if (audioPaused_)
		{
			audio->Play();
			audioPaused_ = false;
		}
	}
}

void Engine::updateRenderTimeTimer()
{
	renderTimer_.SetTimeoutDuration(renderTimeGoalUs_, false);
}

void Engine::updateUpdateTimeTimer()
{
	updateTimer_.SetTimeoutDuration(updateTimeGoalUs_, false);
}
}
