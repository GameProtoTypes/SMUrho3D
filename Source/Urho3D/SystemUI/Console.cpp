//
// Copyright (c) 2017 the Urho3D project.
// Copyright (c) 2008-2015 the Urho3D project.
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

#include "Urho3D/Core/Context.h"
#include "Urho3D/Core/CoreEvents.h"
#include "Urho3D/Engine/EngineEvents.h"
#include "Urho3D/Graphics/Graphics.h"
#include "Urho3D/Graphics/GraphicsEvents.h"
#include "Urho3D/Input/Input.h"
#include "Urho3D/IO/IOEvents.h"
#include "Urho3D/IO/Log.h"
#include "Urho3D/Resource/ResourceCache.h"
#include "SystemUI.h"
#include "SystemUIEvents.h"
#include "Console.h"

#include "Urho3D/DebugNew.h"

namespace Urho3D
{

Console::Console(Context* context) :
    Object(context)
{
    inputBuffer_[0] = 0;

    SetNumHistoryRows(historyRows_);
    VariantMap dummy;
    HandleScreenMode(nullptr, dummy);
    RefreshInterpreters();

    SubscribeToEvent(E_SCREENMODE, URHO3D_HANDLER(Console, HandleScreenMode));
    SubscribeToEvent(E_LOGMESSAGE, URHO3D_HANDLER(Console, HandleLogMessage));
}

Console::~Console()
{
    UnsubscribeFromAllEvents();
}

void Console::SetVisible(bool enable)
{
    isOpen_ = enable;
    if (isOpen_)
    {
        focusInput_ = true;
        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(Console, RenderUi));
    }
    else
    {
        UnsubscribeFromEvent(E_UPDATE);
        ui::SetWindowFocus(nullptr);
    }
}

void Console::Toggle()
{
    SetVisible(!IsVisible());
}

void Console::SetNumHistoryRows(unsigned rows)
{
    historyRows_ = rows;
    if (history_.Size() > rows)
        history_.Resize(rows);
}

bool Console::IsVisible() const
{
    return isOpen_;
}

void Console::RefreshInterpreters()
{
    interpreters_.Clear();
    interpretersPointers_.Clear();

    EventReceiverGroup* group = context_->GetEventReceivers(E_CONSOLECOMMAND);
    if (!group || group->receivers_.Empty())
        return;

    String currentInterpreterName;
    if (currentInterpreter_ < interpreters_.Size())
        currentInterpreterName = interpreters_[currentInterpreter_];

    for (unsigned i = 0; i < group->receivers_.Size(); ++i)
    {
        Object* receiver = group->receivers_[i];
        if (receiver)
        {
            interpreters_.Push(receiver->GetTypeName());
            interpretersPointers_.Push(interpreters_.Back().CString());
        }
    }
    Sort(interpreters_.Begin(), interpreters_.End());

    currentInterpreter_ = interpreters_.IndexOf(currentInterpreterName);
    if (currentInterpreter_ == interpreters_.Size())
        currentInterpreter_ = 0;
}

void Console::HandleLogMessage(StringHash eventType, VariantMap& eventData)
{
    using namespace LogMessage;

    auto level = (LogLevel)eventData[P_LEVEL].GetInt();
    time_t timestamp = eventData[P_TIME].GetUInt();
    const String& logger = eventData[P_LOGGER].GetString();
    const String& message = eventData[P_MESSAGE].GetString();

    // The message may be multi-line, so split to rows in that case
    Vector<String> rows = message.Split('\n');
    for (const auto& row : rows)
        history_.Push(LogEntry{level, timestamp, logger, row});
    scrollToEnd_ = true;

    if (autoVisibleOnError_ && level == LOG_ERROR && !IsVisible())
        SetVisible(true);
}

void Console::RenderContent()
{
    auto region = ui::GetContentRegionAvail();
    auto showCommandInput = !interpretersPointers_.Empty();
    ui::BeginChild("ConsoleScrollArea", ImVec2(region.x, region.y - (showCommandInput ? 30 : 0)), false,
                   ImGuiWindowFlags_HorizontalScrollbar);

    for (const auto& row : history_)
    {
        if (!levelVisible_[row.level_])
            continue;

        if (loggersHidden_.Contains(row.logger_))
            continue;

        ImColor color;
        const char* debugLevel;
        switch (row.level_)
        {
        case LOG_TRACE:
            debugLevel = "T";
            color = ImColor(135, 135, 135);
            break;
        case LOG_DEBUG:
            debugLevel = "D";
            color = ImColor(200, 200, 200);
            break;
        case LOG_INFO:
            debugLevel = "I";
            color = IM_COL32_WHITE;
            break;
        case LOG_WARNING:
            debugLevel = "W";
            color = ImColor(247, 247, 168);
            break;
        case LOG_ERROR:
            debugLevel = "E";
            color = ImColor(247, 168, 168);
            break;
        default:
            debugLevel = "?";
            color = IM_COL32_WHITE;
            break;
        }

        ui::TextColored(color, "[%s] [%s] [%s] : %s", Time::GetTimeStamp(row.timestamp_, "%H:%M:%S").CString(),
            debugLevel, row.logger_.CString(), row.message_.CString());
    }

    if (scrollToEnd_)
    {
        ui::SetScrollHereY(1.f);
        scrollToEnd_ = false;
    }

    ui::EndChild();

    if (showCommandInput)
    {
        ui::PushItemWidth(110);
        if (ui::Combo("##ConsoleInterpreter", &currentInterpreter_, &interpretersPointers_.Front(),
            interpretersPointers_.Size()))
        {
        }
        ui::PopItemWidth();
        ui::SameLine();
        ui::PushItemWidth(region.x - 120);
        if (focusInput_)
        {
            ui::SetKeyboardFocusHere();
            focusInput_ = false;
        }
        if (ui::InputText("##ConsoleInput", inputBuffer_, sizeof(inputBuffer_), ImGuiInputTextFlags_EnterReturnsTrue))
        {
            focusInput_ = true;
            String line(inputBuffer_);
            if (line.Length() && currentInterpreter_ < interpreters_.Size())
            {
                // Store to history, then clear the lineedit
                URHO3D_LOGINFOF("> %s", line.CString());
                if (history_.Size() > historyRows_)
                    history_.Erase(history_.Begin());
                scrollToEnd_ = true;
                inputBuffer_[0] = 0;

                // Send the command as an event for script subsystem
                using namespace ConsoleCommand;

                VariantMap& newEventData = GetEventDataMap();
                newEventData[P_COMMAND] = line;
                newEventData[P_ID] = interpreters_[currentInterpreter_];
                SendEvent(E_CONSOLECOMMAND, newEventData);
            }
        }
        ui::PopItemWidth();
    }
}

void Console::RenderUi(StringHash eventType, VariantMap& eventData)
{
    Graphics* graphics = GetSubsystem<Graphics>();
    ui::SetNextWindowPos(ImVec2(0, 0));
    bool wasOpen = isOpen_;
    ImVec2 size(graphics->GetWidth(), windowSize_.y_);
    ui::SetNextWindowSize(size);

    auto old_rounding = ui::GetStyle().WindowRounding;
    ui::GetStyle().WindowRounding = 0;
    if (ui::Begin("Debug Console", &isOpen_, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoSavedSettings))
    {
        RenderContent();
    }
    else if (wasOpen)
    {
        SetVisible(false);
        ui::SetWindowFocus(nullptr);
        SendEvent(E_CONSOLECLOSED);
    }

    windowSize_.y_ = ui::GetWindowHeight();

    ui::End();

    ui::GetStyle().WindowRounding = old_rounding;
}

void Console::Clear()
{
    history_.Clear();
}

void Console::SetCommandInterpreter(const String& interpreter)
{
    RefreshInterpreters();

    auto index = interpreters_.IndexOf(interpreter);
    if (index == interpreters_.Size())
        index = 0;
    currentInterpreter_ = index;
}

void Console::HandleScreenMode(StringHash eventType, VariantMap& eventData)
{
    Graphics* graphics = GetSubsystem<Graphics>();
    windowSize_.x_ = Clamp(windowSize_.x_, 0, graphics->GetWidth());
    windowSize_.y_ = Clamp(windowSize_.y_, 0, graphics->GetHeight());
}

StringVector Console::GetLoggers() const
{
    HashSet<String> loggers;
    StringVector loggersVector;

    for (const auto& row : history_)
        loggers.Insert(row.logger_);

    for (const String& logger : loggers)
        loggersVector.EmplaceBack(logger);

    Sort(loggersVector.Begin(), loggersVector.End());
    return loggersVector;
}

void Console::SetLoggerVisible(const String& loggerName, bool visible)
{
    scrollToEnd_ = true;
    if (visible)
        loggersHidden_.Erase(loggerName);
    else
        loggersHidden_.Insert(loggerName);
}

bool Console::GetLoggerVisible(const String& loggerName) const
{
    return !loggersHidden_.Contains(loggerName);
}

void Console::SetLevelVisible(LogLevel level, bool visible)
{
    if (level < LOG_TRACE || level >= LOG_NONE)
        return;

    scrollToEnd_ = true;
    levelVisible_[level] = visible;
}

bool Console::GetLevelVisible(LogLevel level) const
{
    if (level < LOG_TRACE || level >= LOG_NONE)
        return false;

    return levelVisible_[level];
}

}
