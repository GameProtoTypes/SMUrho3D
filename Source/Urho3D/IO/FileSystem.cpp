//
// Copyright (c) 2008-2019 the Urho3D project.
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

#include "../Container/ArrayPtr.h"
#include "../Core/Context.h"
#include "../Core/CoreEvents.h"
#include "../Core/Thread.h"
#include "../Engine/EngineEvents.h"
#include "../IO/File.h"
#include "../IO/FileSystem.h"
#include "../IO/IOEvents.h"
#include "../IO/Log.h"

#ifdef __ANDROID__
#include <SDL/SDL_rwops.h>
#endif

#ifndef MINI_URHO
#include <SDL/SDL_filesystem.h>
#endif

#include <sys/stat.h>
#include <cstdio>

#ifdef _WIN32
#ifndef _MSC_VER
#define _WIN32_IE 0x501
#endif
#include <windows.h>
#include <shellapi.h>
#include <direct.h>
#include <shlobj.h>
#include <sys/types.h>
#include <sys/utime.h>
#else
#include <dirent.h>
#include <cerrno>
#include <unistd.h>
#include <utime.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <spawn.h>
#define MAX_PATH 256
#endif

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#include <crt_externs.h>
#define environ (*_NSGetEnviron())
#endif

extern "C"
{
#ifdef __ANDROID__
const char* SDL_Android_GetFilesDir();
char** SDL_Android_GetFileList(const char* path, int* count);
void SDL_Android_FreeFileList(char*** array, int* count);
#elif defined(IOS) || defined(TVOS)
const char* SDL_IOS_GetResourceDir();
const char* SDL_IOS_GetDocumentsDir();
#endif
}

#include "../DebugNew.h"

namespace Urho3D
{

String specifiedExecutableFile;

int DoSystemCommand(const String& commandLine, bool redirectToLog, Context* context)
{
#if defined(TVOS) || defined(IOS)
    return -1;
#else
#if !defined(__EMSCRIPTEN__) && !defined(MINI_URHO)
    if (!redirectToLog)
#endif
        return system(commandLine.CString());

#if !defined(__EMSCRIPTEN__) && !defined(MINI_URHO)
    // Get a platform-agnostic temporary file name for stderr redirection
    String stderrFilename;
    String adjustedCommandLine(commandLine);
    char* prefPath = SDL_GetPrefPath("urho3d", "temp");
    if (prefPath)
    {
        stderrFilename = String(prefPath) + "command-stderr";
        adjustedCommandLine += " 2>" + stderrFilename;
        SDL_free(prefPath);
    }

#ifdef _MSC_VER
    #define popen _popen
    #define pclose _pclose
#endif

    // Use popen/pclose to capture the stdout and stderr of the command
    FILE* file = popen(adjustedCommandLine.CString(), "r");
    if (!file)
        return -1;

    // Capture the standard output stream
    char buffer[128];
    while (!feof(file))
    {
        if (fgets(buffer, sizeof(buffer), file))
            URHO3D_LOGRAW(String(buffer));
    }
    int exitCode = pclose(file);

    // Capture the standard error stream
    if (!stderrFilename.Empty())
    {
        SharedPtr<File> errFile(new File(context, stderrFilename, FILE_READ));
        while (!errFile->IsEof())
        {
            unsigned numRead = errFile->Read(buffer, sizeof(buffer));
            if (numRead)
                URHO3D_LOGERROR(String(buffer, numRead));
        }
    }

    return exitCode;
#endif
#endif
}

enum SystemRunFlag
{
    SR_DEFAULT,
    SR_WAIT_FOR_EXIT,
    SR_READ_OUTPUT = 1 << 1 | SR_WAIT_FOR_EXIT,
};
URHO3D_FLAGSET(SystemRunFlag, SystemRunFlags);

int DoSystemRun(const String& fileName, const Vector<String>& arguments, SystemRunFlags flags, String& output)
{
#if defined(TVOS) || (defined(__ANDROID__) && __ANDROID_API__ < 28)
    return -1;
#else
    String fixedFileName = GetNativePath(fileName);

#ifdef _WIN32
    // Add .exe extension if no extension defined
    if (GetExtension(fixedFileName).Empty())
        fixedFileName += ".exe";

    String commandLine = "\"" + fixedFileName + "\"";
    for (unsigned i = 0; i < arguments.Size(); ++i)
        commandLine += " \"" + arguments[i] + "\"";

    STARTUPINFOW startupInfo{};
    PROCESS_INFORMATION processInfo{};
    startupInfo.cb = sizeof(startupInfo);

    WString commandLineW(commandLine);
    DWORD processFlags = 0;
    if (flags & SR_WAIT_FOR_EXIT)
        // If we are waiting for process result we are likely reading stdout, in that case we probably do not want to see a console window.
        processFlags = CREATE_NO_WINDOW;

    HANDLE pipeRead = 0, pipeWrite = 0;
    if (flags & SR_READ_OUTPUT)
    {
        SECURITY_ATTRIBUTES attr;
        attr.nLength = sizeof(SECURITY_ATTRIBUTES);
        attr.bInheritHandle = FALSE;
        attr.lpSecurityDescriptor = NULL;
        if (!CreatePipe(&pipeRead, &pipeWrite, &attr, 0))
            return -1;

        if (!SetHandleInformation(pipeRead, HANDLE_FLAG_INHERIT, 0))
            return -1;

        if (!SetHandleInformation(pipeWrite, HANDLE_FLAG_INHERIT, 1))
            return -1;

        DWORD mode = PIPE_NOWAIT;
        if (!SetNamedPipeHandleState(pipeRead, &mode, nullptr, nullptr))
            return -1;

        startupInfo.hStdOutput = pipeWrite;
        startupInfo.hStdError = pipeWrite;
        startupInfo.dwFlags |= STARTF_USESTDHANDLES;
    }

    if (!CreateProcessW(nullptr, (wchar_t*)commandLineW.CString(), nullptr, nullptr, TRUE, processFlags, nullptr, nullptr, &startupInfo, &processInfo))
        return -1;

    DWORD exitCode = 0;
    if (flags & SR_WAIT_FOR_EXIT)
    {
        WaitForSingleObject(processInfo.hProcess, INFINITE);
        GetExitCodeProcess(processInfo.hProcess, &exitCode);
    }

    if (flags & SR_READ_OUTPUT)
    {
        char buf[1024];
        for (;;)
        {
            DWORD bytesRead = 0;
            if (!ReadFile(pipeRead, buf, sizeof(buf), &bytesRead, nullptr) || !bytesRead)
                break;
            auto err = GetLastError();

            unsigned start = output.Length();
            output.Resize(start + bytesRead);
            memcpy(&output[start], buf, bytesRead);
        }

        CloseHandle(pipeWrite);
        CloseHandle(pipeRead);
    }

    CloseHandle(processInfo.hProcess);
    CloseHandle(processInfo.hThread);

    return exitCode;
#else

    int desc[2];
    if (flags & SR_READ_OUTPUT)
    {
        if (pipe(desc) == -1)
            return -1;
        fcntl(desc[0], F_SETFL, O_NONBLOCK);
        fcntl(desc[1], F_SETFL, O_NONBLOCK);
    }

    PODVector<const char*> argPtrs;
    argPtrs.Push(fixedFileName.CString());
    for (unsigned i = 0; i < arguments.Size(); ++i)
        argPtrs.Push(arguments[i].CString());
    argPtrs.Push(nullptr);

    pid_t pid = 0;
    posix_spawn_file_actions_t actions{};
    posix_spawn_file_actions_init(&actions);
    if (flags & SR_READ_OUTPUT)
    {
        posix_spawn_file_actions_addclose(&actions, STDOUT_FILENO);
        posix_spawn_file_actions_adddup2(&actions, desc[1], STDOUT_FILENO);
        posix_spawn_file_actions_addclose(&actions, STDERR_FILENO);
        posix_spawn_file_actions_adddup2(&actions, desc[1], STDERR_FILENO);
    }
    posix_spawnp(&pid, fixedFileName.CString(), &actions, 0, (char**)&argPtrs[0], environ);
    posix_spawn_file_actions_destroy(&actions);

    if (pid > 0)
    {
        int exitCode = 0;
        if (flags & SR_WAIT_FOR_EXIT)
            waitpid(pid, &exitCode, 0);

        if (flags & SR_READ_OUTPUT)
        {
            char buf[1024];
            for (;;)
            {
                ssize_t bytesRead = read(desc[0], buf, sizeof(buf));
                if (bytesRead <= 0)
                    break;

                unsigned start = output.Length();
                output.Resize(start + bytesRead);
                memcpy(&output[start], buf, bytesRead);
            }
            close(desc[0]);
            close(desc[1]);
        }
        return exitCode;
    }
    else
        return -1;
#endif
#endif
}

/// Base class for async execution requests.
class AsyncExecRequest : public Thread
{
public:
    /// Construct.
    explicit AsyncExecRequest(unsigned& requestID) :
        requestID_(requestID)
    {
        // Increment ID for next request
        ++requestID;
        if (requestID == M_MAX_UNSIGNED)
            requestID = 1;
    }

    /// Return request ID.
    unsigned GetRequestID() const { return requestID_; }

    /// Return exit code. Valid when IsCompleted() is true.
    int GetExitCode() const { return exitCode_; }

    /// Return completion status.
    bool IsCompleted() const { return completed_; }

protected:
    /// Request ID.
    unsigned requestID_{};
    /// Exit code.
    int exitCode_{};
    /// Completed flag.
    volatile bool completed_{};
};

/// Async system command operation.
class AsyncSystemCommand : public AsyncExecRequest
{
public:
    /// Construct and run.
    AsyncSystemCommand(unsigned requestID, const String& commandLine) :
        AsyncExecRequest(requestID),
        commandLine_(commandLine)
    {
        Run();
    }

    /// The function to run in the thread.
    void ThreadFunction() override
    {
        exitCode_ = DoSystemCommand(commandLine_, false, nullptr);
        completed_ = true;
    }

private:
    /// Command line.
    String commandLine_;
};

/// Async system run operation.
class AsyncSystemRun : public AsyncExecRequest
{
public:
    /// Construct and run.
    AsyncSystemRun(unsigned requestID, const String& fileName, const Vector<String>& arguments) :
        AsyncExecRequest(requestID),
        fileName_(fileName),
        arguments_(arguments)
    {
        Run();
    }

    /// The function to run in the thread.
    void ThreadFunction() override
    {
        String output;
        exitCode_ = DoSystemRun(fileName_, arguments_, SR_WAIT_FOR_EXIT, output);
        completed_ = true;
    }

private:
    /// File to run.
    String fileName_;
    /// Command line split in arguments.
    const Vector<String>& arguments_;
};

FileSystem::FileSystem(Context* context) :
    Object(context)
{
    SubscribeToEvent(E_BEGINFRAME, URHO3D_HANDLER(FileSystem, HandleBeginFrame));
}

FileSystem::~FileSystem()
{
    // If any async exec items pending, delete them
    if (asyncExecQueue_.Size())
    {
        for (List<AsyncExecRequest*>::Iterator i = asyncExecQueue_.Begin(); i != asyncExecQueue_.End(); ++i)
            delete(*i);

        asyncExecQueue_.Clear();
    }
}

bool FileSystem::SetCurrentDir(const String& pathName)
{
    if (!CheckAccess(pathName))
    {
        URHO3D_LOGERROR("Access denied to " + pathName);
        return false;
    }
#ifdef _WIN32
    if (SetCurrentDirectoryW(GetWideNativePath(pathName).CString()) == FALSE)
    {
        URHO3D_LOGERROR("Failed to change directory to " + pathName);
        return false;
    }
#else
    if (chdir(GetNativePath(pathName).CString()) != 0)
    {
        URHO3D_LOGERROR("Failed to change directory to " + pathName);
        return false;
    }
#endif

    return true;
}

bool FileSystem::CreateDir(const String& pathName)
{
    if (!CheckAccess(pathName))
    {
        URHO3D_LOGERROR("Access denied to " + pathName);
        return false;
    }

    // Create each of the parents if necessary
    String parentPath = GetParentPath(pathName);
    if (parentPath.Length() > 1 && !DirExists(parentPath))
    {
        if (!CreateDir(parentPath))
            return false;
    }

#ifdef _WIN32
    bool success = (CreateDirectoryW(GetWideNativePath(RemoveTrailingSlash(pathName)).CString(), nullptr) == TRUE) ||
        (GetLastError() == ERROR_ALREADY_EXISTS);
#else
    bool success = mkdir(GetNativePath(RemoveTrailingSlash(pathName)).CString(), S_IRWXU) == 0 || errno == EEXIST;
#endif

    if (success)
        URHO3D_LOGDEBUG("Created directory " + pathName);
    else
        URHO3D_LOGERROR("Failed to create directory " + pathName);

    return success;
}

void FileSystem::SetExecuteConsoleCommands(bool enable)
{
    if (enable == executeConsoleCommands_)
        return;

    executeConsoleCommands_ = enable;
    if (enable)
        SubscribeToEvent(E_CONSOLECOMMAND, URHO3D_HANDLER(FileSystem, HandleConsoleCommand));
    else
        UnsubscribeFromEvent(E_CONSOLECOMMAND);
}

int FileSystem::SystemCommand(const String& commandLine, bool redirectStdOutToLog)
{
    if (allowedPaths_.Empty())
        return DoSystemCommand(commandLine, redirectStdOutToLog, context_);
    else
    {
        URHO3D_LOGERROR("Executing an external command is not allowed");
        return -1;
    }
}

int FileSystem::SystemRun(const String& fileName, const Vector<String>& arguments, String& output)
{
    if (allowedPaths_.Empty())
        return DoSystemRun(fileName, arguments, SR_READ_OUTPUT, output);
    else
    {
        URHO3D_LOGERROR("Executing an external command is not allowed");
        return -1;
    }
}

int FileSystem::SystemRun(const String& fileName, const Vector<String>& arguments)
{
    String output;
    return SystemRun(fileName, arguments, output);
}

int FileSystem::SystemSpawn(const String& fileName, const Vector<String>& arguments)
{
    String output;
    if (allowedPaths_.Empty())
        return DoSystemRun(fileName, arguments, SR_DEFAULT, output);
    else
    {
        URHO3D_LOGERROR("Executing an external command is not allowed");
        return -1;
    }
}

unsigned FileSystem::SystemCommandAsync(const String& commandLine)
{
#ifdef URHO3D_THREADING
    if (allowedPaths_.Empty())
    {
        unsigned requestID = nextAsyncExecID_;
        auto* cmd = new AsyncSystemCommand(nextAsyncExecID_, commandLine);
        asyncExecQueue_.Push(cmd);
        return requestID;
    }
    else
    {
        URHO3D_LOGERROR("Executing an external command is not allowed");
        return M_MAX_UNSIGNED;
    }
#else
    URHO3D_LOGERROR("Can not execute an asynchronous command as threading is disabled");
    return M_MAX_UNSIGNED;
#endif
}

unsigned FileSystem::SystemRunAsync(const String& fileName, const Vector<String>& arguments)
{
#ifdef URHO3D_THREADING
    if (allowedPaths_.Empty())
    {
        unsigned requestID = nextAsyncExecID_;
        auto* cmd = new AsyncSystemRun(nextAsyncExecID_, fileName, arguments);
        asyncExecQueue_.Push(cmd);
        return requestID;
    }
    else
    {
        URHO3D_LOGERROR("Executing an external command is not allowed");
        return M_MAX_UNSIGNED;
    }
#else
    URHO3D_LOGERROR("Can not run asynchronously as threading is disabled");
    return M_MAX_UNSIGNED;
#endif
}

bool FileSystem::SystemOpen(const String& fileName, const String& mode)
{
    if (allowedPaths_.Empty())
    {
        // allow opening of http and file urls
        if (!fileName.StartsWith("http://") && !fileName.StartsWith("https://") && !fileName.StartsWith("file://"))
        {
            if (!FileExists(fileName) && !DirExists(fileName))
            {
                URHO3D_LOGERROR("File or directory " + fileName + " not found");
                return false;
            }
        }

#ifdef _WIN32
        bool success = (size_t)ShellExecuteW(nullptr, !mode.Empty() ? WString(mode).CString() : nullptr,
            GetWideNativePath(fileName).CString(), nullptr, nullptr, SW_SHOW) > 32;
#else
        Vector<String> arguments;
        arguments.Push(fileName);
        bool success = SystemRun(
#if defined(__APPLE__)
            "/usr/bin/open",
#else
            "/usr/bin/xdg-open",
#endif
            arguments) == 0;
#endif
        if (!success)
            URHO3D_LOGERROR("Failed to open " + fileName + " externally");
        return success;
    }
    else
    {
        URHO3D_LOGERROR("Opening a file externally is not allowed");
        return false;
    }
}

bool FileSystem::Copy(const String& srcFileName, const String& destFileName)
{
    if (!CheckAccess(GetPath(srcFileName)))
    {
        URHO3D_LOGERROR("Access denied to " + srcFileName);
        return false;
    }
    if (!CheckAccess(GetPath(destFileName)))
    {
        URHO3D_LOGERROR("Access denied to " + destFileName);
        return false;
    }

    SharedPtr<File> srcFile(new File(context_, srcFileName, FILE_READ));
    if (!srcFile->IsOpen())
        return false;
    SharedPtr<File> destFile(new File(context_, destFileName, FILE_WRITE));
    if (!destFile->IsOpen())
        return false;

    unsigned fileSize = srcFile->GetSize();
    SharedArrayPtr<unsigned char> buffer(new unsigned char[fileSize]);

    unsigned bytesRead = srcFile->Read(buffer.Get(), fileSize);
    unsigned bytesWritten = destFile->Write(buffer.Get(), fileSize);
    return bytesRead == fileSize && bytesWritten == fileSize;
}

bool FileSystem::Rename(const String& srcFileName, const String& destFileName)
{
    if (!CheckAccess(GetPath(srcFileName)))
    {
        URHO3D_LOGERROR("Access denied to " + srcFileName);
        return false;
    }
    if (!CheckAccess(GetPath(destFileName)))
    {
        URHO3D_LOGERROR("Access denied to " + destFileName);
        return false;
    }

#ifdef _WIN32
    return MoveFileW(GetWideNativePath(srcFileName).CString(), GetWideNativePath(destFileName).CString()) != 0;
#else
    return rename(GetNativePath(srcFileName).CString(), GetNativePath(destFileName).CString()) == 0;
#endif
}

bool FileSystem::Delete(const String& fileName)
{
    if (!CheckAccess(GetPath(fileName)))
    {
        URHO3D_LOGERROR("Access denied to " + fileName);
        return false;
    }

#ifdef _WIN32
    return DeleteFileW(GetWideNativePath(fileName).CString()) != 0;
#else
    return remove(GetNativePath(fileName).CString()) == 0;
#endif
}

String FileSystem::GetCurrentDir() const
{
#ifdef _WIN32
    wchar_t path[MAX_PATH];
    path[0] = 0;
    GetCurrentDirectoryW(MAX_PATH, path);
    return AddTrailingSlash(String(path));
#else
    char path[MAX_PATH];
    path[0] = 0;
    getcwd(path, MAX_PATH);
    return AddTrailingSlash(String(path));
#endif
}

bool FileSystem::CheckAccess(const String& pathName) const
{
    String fixedPath = AddTrailingSlash(pathName);

    // If no allowed directories defined, succeed always
    if (allowedPaths_.Empty())
        return true;

    // If there is any attempt to go to a parent directory, disallow
    if (fixedPath.Contains(".."))
        return false;

    // Check if the path is a partial match of any of the allowed directories
    for (HashSet<String>::ConstIterator i = allowedPaths_.Begin(); i != allowedPaths_.End(); ++i)
    {
        if (fixedPath.Find(*i) == 0)
            return true;
    }

    // Not found, so disallow
    return false;
}

unsigned FileSystem::GetLastModifiedTime(const String& fileName) const
{
    if (fileName.Empty() || !CheckAccess(fileName))
        return 0;

#ifdef _WIN32
    struct _stat st;
    if (!_stat(fileName.CString(), &st))
        return (unsigned)st.st_mtime;
    else
        return 0;
#else
    struct stat st{};
    if (!stat(fileName.CString(), &st))
        return (unsigned)st.st_mtime;
    else
        return 0;
#endif
}

bool FileSystem::FileExists(const String& fileName) const
{
    if (!CheckAccess(GetPath(fileName)))
        return false;

#ifdef __ANDROID__
    if (URHO3D_IS_ASSET(fileName))
    {
        SDL_RWops* rwOps = SDL_RWFromFile(URHO3D_ASSET(fileName), "rb");
        if (rwOps)
        {
            SDL_RWclose(rwOps);
            return true;
        }
        else
            return false;
    }
#endif

    String fixedName = GetNativePath(RemoveTrailingSlash(fileName));

#ifdef _WIN32
    DWORD attributes = GetFileAttributesW(WString(fixedName).CString());
    if (attributes == INVALID_FILE_ATTRIBUTES || attributes & FILE_ATTRIBUTE_DIRECTORY)
        return false;
#else
    struct stat st{};
    if (stat(fixedName.CString(), &st) || st.st_mode & S_IFDIR)
        return false;
#endif

    return true;
}

bool FileSystem::DirExists(const String& pathName) const
{
    if (!CheckAccess(pathName))
        return false;

#ifndef _WIN32
    // Always return true for the root directory
    if (pathName == "/")
        return true;
#endif

    String fixedName = GetNativePath(RemoveTrailingSlash(pathName));

#ifdef __ANDROID__
    if (URHO3D_IS_ASSET(fixedName))
    {
        // Split the pathname into two components: the longest parent directory path and the last name component
        String assetPath(URHO3D_ASSET((fixedName + "/")));
        String parentPath;
        unsigned pos = assetPath.FindLast('/', assetPath.Length() - 2);
        if (pos != String::NPOS)
        {
            parentPath = assetPath.Substring(0, pos);
            assetPath = assetPath.Substring(pos + 1);
        }
        assetPath.Resize(assetPath.Length() - 1);

        bool exist = false;
        int count;
        char** list = SDL_Android_GetFileList(parentPath.CString(), &count);
        for (int i = 0; i < count; ++i)
        {
            exist = assetPath == list[i];
            if (exist)
                break;
        }
        SDL_Android_FreeFileList(&list, &count);
        return exist;
    }
#endif

#ifdef _WIN32
    DWORD attributes = GetFileAttributesW(WString(fixedName).CString());
    if (attributes == INVALID_FILE_ATTRIBUTES || !(attributes & FILE_ATTRIBUTE_DIRECTORY))
        return false;
#else
    struct stat st{};
    if (stat(fixedName.CString(), &st) || !(st.st_mode & S_IFDIR))
        return false;
#endif

    return true;
}

void FileSystem::ScanDir(Vector<String>& result, const String& pathName, const String& filter, unsigned flags, bool recursive) const
{
    result.Clear();

    if (CheckAccess(pathName))
    {
        String initialPath = AddTrailingSlash(pathName);
        ScanDirInternal(result, initialPath, initialPath, filter, flags, recursive);
    }
}

String FileSystem::GetProgramDir() const
{
#if defined(__ANDROID__)
    // This is an internal directory specifier pointing to the assets in the .apk
    // Files from this directory will be opened using special handling
    return APK;
#elif defined(IOS) || defined(TVOS)
    return AddTrailingSlash(SDL_IOS_GetResourceDir());
#elif DESKTOP
    return GetPath(GetProgramFileName());
#else
    return GetCurrentDir();
#endif
}
#if DESKTOP
String FileSystem::GetProgramFileName() const
{
    if (!specifiedExecutableFile.Empty())
        return specifiedExecutableFile;

    return GetInterpreterFileName();
}

String FileSystem::GetInterpreterFileName() const
{
#if defined(_WIN32)
    wchar_t exeName[MAX_PATH];
    exeName[0] = 0;
    GetModuleFileNameW(nullptr, exeName, MAX_PATH);
    return String(exeName);
#elif defined(__APPLE__)
    char exeName[MAX_PATH];
    memset(exeName, 0, MAX_PATH);
    unsigned size = MAX_PATH;
    _NSGetExecutablePath(exeName, &size);
    return String(exeName);
#elif defined(__linux__)
    char exeName[MAX_PATH];
    memset(exeName, 0, MAX_PATH);
    pid_t pid = getpid();
    String link = "/proc/" + String(pid) + "/exe";
    readlink(link.CString(), exeName, MAX_PATH);
    return String(exeName);
#else
    return String();
#endif
}
#endif
String FileSystem::GetUserDocumentsDir() const
{
#if defined(__ANDROID__)
    return AddTrailingSlash(SDL_Android_GetFilesDir());
#elif defined(IOS) || defined(TVOS)
    return AddTrailingSlash(SDL_IOS_GetDocumentsDir());
#elif defined(_WIN32)
    wchar_t pathName[MAX_PATH];
    pathName[0] = 0;
    SHGetSpecialFolderPathW(nullptr, pathName, CSIDL_PERSONAL, 0);
    return AddTrailingSlash(String(pathName));
#else
    char pathName[MAX_PATH];
    pathName[0] = 0;
    strcpy(pathName, getenv("HOME"));
    return AddTrailingSlash(String(pathName));
#endif
}

String FileSystem::GetAppPreferencesDir(const String& org, const String& app) const
{
    String dir;
#ifndef MINI_URHO
    char* prefPath = SDL_GetPrefPath(org.CString(), app.CString());
    if (prefPath)
    {
        dir = GetInternalPath(String(prefPath));
        SDL_free(prefPath);
    }
    else
#endif
        URHO3D_LOGWARNING("Could not get application preferences directory");

    return dir;
}

void FileSystem::RegisterPath(const String& pathName)
{
    if (pathName.Empty())
        return;

    allowedPaths_.Insert(AddTrailingSlash(pathName));
}

bool FileSystem::SetLastModifiedTime(const String& fileName, unsigned newTime)
{
    if (fileName.Empty() || !CheckAccess(fileName))
        return false;

#ifdef _WIN32
    struct _stat oldTime;
    struct _utimbuf newTimes;
    if (_stat(fileName.CString(), &oldTime) != 0)
        return false;
    newTimes.actime = oldTime.st_atime;
    newTimes.modtime = newTime;
    return _utime(fileName.CString(), &newTimes) == 0;
#else
    struct stat oldTime{};
    struct utimbuf newTimes{};
    if (stat(fileName.CString(), &oldTime) != 0)
        return false;
    newTimes.actime = oldTime.st_atime;
    newTimes.modtime = newTime;
    return utime(fileName.CString(), &newTimes) == 0;
#endif
}

void FileSystem::ScanDirInternal(Vector<String>& result, String path, const String& startPath,
    const String& filter, unsigned flags, bool recursive) const
{
    path = AddTrailingSlash(path);
    String deltaPath;
    if (path.Length() > startPath.Length())
        deltaPath = path.Substring(startPath.Length());

    String filterExtension = filter.Substring(filter.FindLast('.'));
    if (filterExtension.Contains('*'))
        filterExtension.Clear();

#ifdef __ANDROID__
    if (URHO3D_IS_ASSET(path))
    {
        String assetPath(URHO3D_ASSET(path));
        assetPath.Resize(assetPath.Length() - 1);       // AssetManager.list() does not like trailing slash
        int count;
        char** list = SDL_Android_GetFileList(assetPath.CString(), &count);
        for (int i = 0; i < count; ++i)
        {
            String fileName(list[i]);
            if (!(flags & SCAN_HIDDEN) && fileName.StartsWith("."))
                continue;

#ifdef ASSET_DIR_INDICATOR
            // Patch the directory name back after retrieving the directory flag
            bool isDirectory = fileName.EndsWith(ASSET_DIR_INDICATOR);
            if (isDirectory)
            {
                fileName.Resize(fileName.Length() - sizeof(ASSET_DIR_INDICATOR) / sizeof(char) + 1);
                if (flags & SCAN_DIRS)
                    result.Push(deltaPath + fileName);
                if (recursive)
                    ScanDirInternal(result, path + fileName, startPath, filter, flags, recursive);
            }
            else if (flags & SCAN_FILES)
#endif
            {
                if (filterExtension.Empty() || fileName.EndsWith(filterExtension))
                    result.Push(deltaPath + fileName);
            }
        }
        SDL_Android_FreeFileList(&list, &count);
        return;
    }
#endif
#ifdef _WIN32
    WIN32_FIND_DATAW info;
    HANDLE handle = FindFirstFileW(WString(path + "*").CString(), &info);
    if (handle != INVALID_HANDLE_VALUE)
    {
        do
        {
            String fileName(info.cFileName);
            if (!fileName.Empty())
            {
                if (info.dwFileAttributes & FILE_ATTRIBUTE_HIDDEN && !(flags & SCAN_HIDDEN))
                    continue;
                if (info.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
                {
                    if (flags & SCAN_DIRS)
                        result.Push(deltaPath + fileName);
                    if (recursive && fileName != "." && fileName != "..")
                        ScanDirInternal(result, path + fileName, startPath, filter, flags, recursive);
                }
                else if (flags & SCAN_FILES)
                {
                    if (filterExtension.Empty() || fileName.EndsWith(filterExtension))
                        result.Push(deltaPath + fileName);
                }
            }
        }
        while (FindNextFileW(handle, &info));

        FindClose(handle);
    }
#else
    DIR* dir;
    struct dirent* de;
    struct stat st{};
    dir = opendir(GetNativePath(path).CString());
    if (dir)
    {
        while ((de = readdir(dir)))
        {
            /// \todo Filename may be unnormalized Unicode on Mac OS X. Re-normalize as necessary
            String fileName(de->d_name);
            bool normalEntry = fileName != "." && fileName != "..";
            if (normalEntry && !(flags & SCAN_HIDDEN) && fileName.StartsWith("."))
                continue;
            String pathAndName = path + fileName;
            if (!stat(pathAndName.CString(), &st))
            {
                if (st.st_mode & S_IFDIR)
                {
                    if (flags & SCAN_DIRS)
                        result.Push(deltaPath + fileName);
                    if (recursive && normalEntry)
                        ScanDirInternal(result, path + fileName, startPath, filter, flags, recursive);
                }
                else if (flags & SCAN_FILES)
                {
                    if (filterExtension.Empty() || fileName.EndsWith(filterExtension))
                        result.Push(deltaPath + fileName);
                }
            }
        }
        closedir(dir);
    }
#endif
}

void FileSystem::HandleBeginFrame(StringHash eventType, VariantMap& eventData)
{
    /// Go through the execution queue and post + remove completed requests
    for (List<AsyncExecRequest*>::Iterator i = asyncExecQueue_.Begin(); i != asyncExecQueue_.End();)
    {
        AsyncExecRequest* request = *i;
        if (request->IsCompleted())
        {
            using namespace AsyncExecFinished;

            VariantMap& newEventData = GetEventDataMap();
            newEventData[P_REQUESTID] = request->GetRequestID();
            newEventData[P_EXITCODE] = request->GetExitCode();
            SendEvent(E_ASYNCEXECFINISHED, newEventData);

            delete request;
            i = asyncExecQueue_.Erase(i);
        }
        else
            ++i;
    }
}

void FileSystem::HandleConsoleCommand(StringHash eventType, VariantMap& eventData)
{
    using namespace ConsoleCommand;
    if (eventData[P_ID].GetString() == GetTypeName())
        SystemCommand(eventData[P_COMMAND].GetString(), true);
}

void SplitPath(const String& fullPath, String& pathName, String& fileName, String& extension, bool lowercaseExtension)
{
    String fullPathCopy = GetInternalPath(fullPath);

    unsigned extPos = fullPathCopy.FindLast('.');
    unsigned pathPos = fullPathCopy.FindLast('/');

    if (extPos != String::NPOS && (pathPos == String::NPOS || extPos > pathPos))
    {
        extension = fullPathCopy.Substring(extPos);
        if (lowercaseExtension)
            extension = extension.ToLower();
        fullPathCopy = fullPathCopy.Substring(0, extPos);
    }
    else
        extension.Clear();

    pathPos = fullPathCopy.FindLast('/');
    if (pathPos != String::NPOS)
    {
        fileName = fullPathCopy.Substring(pathPos + 1);
        pathName = fullPathCopy.Substring(0, pathPos + 1);
    }
    else
    {
        fileName = fullPathCopy;
        pathName.Clear();
    }
}

String GetPath(const String& fullPath)
{
    String path, file, extension;
    SplitPath(fullPath, path, file, extension);
    return path;
}

String GetFileName(const String& fullPath)
{
    String path, file, extension;
    SplitPath(fullPath, path, file, extension);
    return file;
}

String GetExtension(const String& fullPath, bool lowercaseExtension)
{
    String path, file, extension;
    SplitPath(fullPath, path, file, extension, lowercaseExtension);
    return extension;
}

String GetFileNameAndExtension(const String& fileName, bool lowercaseExtension)
{
    String path, file, extension;
    SplitPath(fileName, path, file, extension, lowercaseExtension);
    return file + extension;
}

String ReplaceExtension(const String& fullPath, const String& newExtension)
{
    String path, file, extension;
    SplitPath(fullPath, path, file, extension);
    return path + file + newExtension;
}

String AddTrailingSlash(const String& pathName)
{
    String ret = pathName.Trimmed();
    ret.Replace('\\', '/');
    if (!ret.Empty() && ret.Back() != '/')
        ret += '/';
    return ret;
}

String RemoveTrailingSlash(const String& pathName)
{
    String ret = pathName.Trimmed();
    ret.Replace('\\', '/');
    if (!ret.Empty() && ret.Back() == '/')
        ret.Resize(ret.Length() - 1);
    return ret;
}

String GetParentPath(const String& path)
{
    unsigned pos = RemoveTrailingSlash(path).FindLast('/');
    if (pos != String::NPOS)
        return path.Substring(0, pos + 1);
    else
        return String();
}

String GetInternalPath(const String& pathName)
{
    return pathName.Replaced('\\', '/');
}

String GetNativePath(const String& pathName)
{
#ifdef _WIN32
    return pathName.Replaced('/', '\\');
#else
    return pathName;
#endif
}

WString GetWideNativePath(const String& pathName)
{
#ifdef _WIN32
    return WString(pathName.Replaced('/', '\\'));
#else
    return WString(pathName);
#endif
}

bool IsAbsolutePath(const String& pathName)
{
    if (pathName.Empty())
        return false;

    String path = GetInternalPath(pathName);

    if (path[0] == '/')
        return true;

#ifdef _WIN32
    if (path.Length() > 1 && IsAlpha(path[0]) && path[1] == ':')
        return true;
#endif

    return false;
}

bool FileSystem::CreateDirs(const String& root, const String& subdirectory)
{
    String folder = AddTrailingSlash(GetInternalPath(root));
    String sub = GetInternalPath(subdirectory);
    Vector<String> subs = sub.Split('/');

    for (unsigned i = 0; i < subs.Size(); i++)
    {
        folder += subs[i];
        folder += "/";

        if (DirExists(folder))
            continue;

        CreateDir(folder);

        if (!DirExists(folder))
            return false;
    }

    return true;

}

bool FileSystem::CreateDirsRecursive(const String& directoryIn)
{
    String directory = AddTrailingSlash(GetInternalPath(directoryIn));

    if (DirExists(directory))
        return true;

    if (FileExists(directory))
        return false;

    String parentPath = directory;

    Vector<String> paths;

    paths.Push(directory);

    while (true)
    {
        parentPath = GetParentPath(parentPath);

        if (!parentPath.Length())
            break;

        paths.Push(parentPath);
    }

    if (!paths.Size())
        return false;

    for (auto i = (int) (paths.Size() - 1); i >= 0; i--)
    {
        const String& pathName = paths[i];

        if (FileExists(pathName))
            return false;

        if (DirExists(pathName))
            continue;

        if (!CreateDir(pathName))
            return false;

        // double check
        if (!DirExists(pathName))
            return false;

    }

    return true;

}

bool FileSystem::RemoveDir(const String& directoryIn, bool recursive)
{
    String directory = AddTrailingSlash(directoryIn);

    if (!DirExists(directory))
        return false;

    Vector<String> results;

    // ensure empty if not recursive
    if (!recursive)
    {
        ScanDir(results, directory, "*", SCAN_DIRS | SCAN_FILES | SCAN_HIDDEN, true );
        while (results.Remove(".")) {}
        while (results.Remove("..")) {}

        if (results.Size())
            return false;

#ifdef WIN32
        return RemoveDirectoryW(GetWideNativePath(directory).CString()) != 0;
#else
        return remove(GetNativePath(directory).CString()) == 0;
#endif
    }

    // delete all files at this level
    ScanDir(results, directory, "*", SCAN_FILES | SCAN_HIDDEN, false );
    for (unsigned i = 0; i < results.Size(); i++)
    {
        if (!Delete(directory + results[i]))
            return false;
    }
    results.Clear();

    // recurse into subfolders
    ScanDir(results, directory, "*", SCAN_DIRS, false );
    for (unsigned i = 0; i < results.Size(); i++)
    {
        if (results[i] == "." || results[i] == "..")
            continue;

        if (!RemoveDir(directory + results[i], true))
            return false;
    }

    return RemoveDir(directory, false);

}

bool FileSystem::CopyDir(const String& directoryIn, const String& directoryOut)
{
    if (FileExists(directoryOut))
        return false;

    Vector<String> results;
    ScanDir(results, directoryIn, "*", SCAN_FILES, true );

    for (unsigned i = 0; i < results.Size(); i++)
    {
        String srcFile = directoryIn + "/" + results[i];
        String dstFile = directoryOut + "/" + results[i];

        String dstPath = GetPath(dstFile);

        if (!CreateDirsRecursive(dstPath))
            return false;

        //LOGINFOF("SRC: %s DST: %s", srcFile.CString(), dstFile.CString());
        if (!Copy(srcFile, dstFile))
            return false;
    }

    return true;

}

bool IsAbsoluteParentPath(const String& absParentPath, const String& fullPath)
{
    if (!IsAbsolutePath(absParentPath) || !IsAbsolutePath(fullPath))
        return false;

    String path1 = AddTrailingSlash(GetSanitizedPath(absParentPath));
    String path2 = AddTrailingSlash(GetSanitizedPath(GetPath(fullPath)));

    if (path2.StartsWith(path1))
        return true;

    return false;
}

String GetSanitizedPath(const String& path)
{
    String sanitized = GetInternalPath(path);
    StringVector parts = sanitized.Split('/');

    bool hasTrailingSlash = path.EndsWith("/") || path.EndsWith("\\");

#ifndef _WIN32

    bool absolute = IsAbsolutePath(path);
    sanitized = String::Joined(parts, "/");
    if (absolute)
        sanitized = "/" + sanitized;

#else

    sanitized = String::Joined(parts, "/");

#endif

    if (hasTrailingSlash)
        sanitized += "/";

    return sanitized;

}

bool GetRelativePath(const String& fromPath, const String& toPath, String& output)
{
    output = String::EMPTY;

    String from = GetSanitizedPath(fromPath);
    String to = GetSanitizedPath(toPath);

    StringVector fromParts = from.Split('/');
    StringVector toParts = to.Split('/');

    if (!fromParts.Size() || !toParts.Size())
        return false;

    if (fromParts == toParts)
    {
        return true;
    }

    // no common base?
    if (fromParts[0] != toParts[0])
        return false;

    int startIdx;

    for (startIdx = 0; startIdx < toParts.Size(); startIdx++)
    {
        if (startIdx >= fromParts.Size() || fromParts[startIdx] != toParts[startIdx])
            break;
    }

    if (startIdx == toParts.Size())
    {
        if (from.EndsWith("/") && to.EndsWith("/"))
        {
            for (unsigned i = 0; i < fromParts.Size() - startIdx; i++)
            {
                output += "../";
            }

            return true;
        }
        return false;
    }

    for (int i = 0; i < (int)fromParts.Size() - startIdx; i++)
    {
        output += "../";
    }

    for (int i = startIdx; i < (int) toParts.Size(); i++)
    {
        output += toParts[i] + "/";
    }

    return true;

}

String FileSystem::GetTemporaryDir() const
{
#if defined(_WIN32)
#if defined(MINI_URHO)
    return getenv("TMP");
#else
    wchar_t pathName[MAX_PATH];
    pathName[0] = 0;
    GetTempPathW(SDL_arraysize(pathName), pathName);
    return AddTrailingSlash(String(pathName));
#endif
#else
    if (char* pathName = getenv("TMPDIR"))
        return AddTrailingSlash(pathName);
#ifdef P_tmpdir
    return AddTrailingSlash(P_tmpdir);
#else
    return "/tmp/";
#endif
#endif
}

String GetAbsolutePath(const String& path)
{
    Vector<String> parts;
#if !_WIN32
    parts.Push("");
#endif
    parts.Push(path.Split('/'));

    int index = 0;
    while (index < parts.Size() - 1)
    {
        if (parts[index] != ".." && parts[index + 1] == "..")
        {
            parts.Erase(index, 2);
            index = Max(0, --index);
        }
        else
            ++index;
    }

    return String::Joined(parts, "/");
}

}
