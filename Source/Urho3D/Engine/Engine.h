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

#pragma once

#include "../Core/Object.h"
#include "../Core/Timer.h"

namespace Urho3D
{

class Console;
class DebugHud;

/// Urho3D engine. Creates the other subsystems.
class URHO3D_API Engine : public Object
{
    URHO3D_OBJECT(Engine, Object);

public:
    /// Construct.
    explicit Engine(Context* context);
    /// Destruct. Free all subsystems.
    ~Engine() override;

    /// Initialize engine using parameters given and show the application window. Return true if successful.
    bool Initialize(const VariantMap& parameters);
    /// Reinitialize resource cache subsystem using parameters given. Implicitly called by Initialize. Return true if successful.
    bool InitializeResourceCache(const VariantMap& parameters, bool removeOld = true);
	
	/// updates the engine - returns approximately how many microseconds until it should be called again.
	unsigned FreeUpdate();
    /// Create the console and return it. May return null if engine configuration does not allow creation (headless mode.)
    Console* CreateConsole();
    /// Create the debug hud.
    DebugHud* CreateDebugHud();

	/// Return how many Renders have occurred.
	long long GetRenderCount() const { return renderTick_; }

	/// Return how many Updates have occured.
	long long GetUpdateCount() const { return updateTick_; }


    float GetUpdateTimeGoalMs() const { return updateTimeGoalUs_ * 1000.0f; }

    float GetRenderTimeGoalMs() const { return renderTimeGoalUs_ * 1000.0f; }

	/// Return the duration in milliseconds of the last render frame.
	float GetLastRenderTimeMs() const { return lastRenderTimeUs_ * 1000.0f; }

	/// Return the duration in milliseconds of the last update frame.
	float GetLastUpdateTimeMs() const { return lastUpdateTimeUs_ * 1000.0f; }




    /// Set fps goal for rendering
    void SetRenderFpsGoal(int fps);
	/// Set render time goal for rendering
	void SetRenderTimeGoalUs(unsigned timeUs);
	/// Set fps goal for update
	void SetUpdateFpsGoal(unsigned fps);
	/// Set the time interval for Update events
	void SetUpdateTimeGoalUs(unsigned updateTimeUs);


    /// Set whether to pause update events and audio when minimized.
    void SetPauseMinimized(bool enable);
    /// Set whether to exit automatically on exit request (window close button.)
    void SetAutoExit(bool enable);
    /// Close the graphics window and set the exit flag. No-op on iOS/tvOS, as an iOS/tvOS application can not legally exit.
    void Exit();
    /// Dump profiling information to the log.
    void DumpProfiler();
    /// Dump information of all resources to the log.
    void DumpResources(bool dumpFileName = false);
    /// Dump information of all memory allocations to the log. Supported in MSVC debug mode only.
    void DumpMemory();

    /// Return whether to pause update events and audio when minimized.
    bool GetPauseMinimized() const { return pauseMinimized_; }

    /// Return whether to exit automatically on exit request.
    bool GetAutoExit() const { return autoExit_; }

    /// Return whether engine has been initialized.
    bool IsInitialized() const { return initialized_; }

    /// Return whether exit has been requested.
    bool IsExiting() const { return exiting_; }

    /// Return whether the engine has been created in headless mode.
    bool IsHeadless() const { return headless_; }



    /// Parse the engine startup parameters map from command line arguments.
    static VariantMap ParseParameters(const Vector<String>& arguments);
    /// Return whether startup parameters contains a specific parameter.
    static bool HasParameter(const VariantMap& parameters, const String& parameter);
    /// Get an engine startup parameter, with default value if missing.
    static const Variant
        & GetParameter(const VariantMap& parameters, const String& parameter, const Variant& defaultValue = Variant::EMPTY);

private:
    /// Handle exit requested event. Auto-exit if enabled.
    void HandleExitRequested(StringHash eventType, VariantMap& eventData);
    /// Actually perform the exit actions.
    void DoExit();

    /// Updates
    void Update();

    void SendUpdateEvents();

    /// Renders
    void Render();



   	void updateAudioPausing();
	
	

    HiresTimer updateTimerTracker_;
	HiresTimer renderTimerTracker_;

	long long renderTick_{ 0 };
	long long updateTick_{ 0 };

	HiresTimer updateTimer_;
	HiresTimer renderGoalTimer_;

	float renderTimeGoalUs_{ 5000.0f };  //200 Hz   
    float updateTimeGoalUs_{ 16666.0f*0.25f }; //60 Hz

    float lastRenderTimeUs_{ 0 };
    float lastUpdateTimeUs_{ 0 };


    /// Pause when minimized flag.
    bool pauseMinimized_;
#ifdef URHO3D_TESTING
    /// Time out counter for testing.
    long long timeOut_;
#endif
    /// Auto-exit flag.
    bool autoExit_;
    /// Initialized flag.
    bool initialized_;
    /// Exiting flag.
    bool exiting_;
    /// Headless mode flag.
    bool headless_;
    /// Audio paused flag.
    bool audioPaused_;

	void updateFpsGoalTimer();
	void updateUpdateTimeTimer();
	void updateAveragingTimeWindows();

};

}
