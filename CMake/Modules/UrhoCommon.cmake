#
# Copyright (c) 2008-2018 the Urho3D project.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

include(ucm)

# Set compiler variable
set ("${CMAKE_CXX_COMPILER_ID}" ON)

# Configure variables
set (URHO3D_URL "https://github.com/urho3d/Urho3D")
set (URHO3D_DESCRIPTION "Urho3D is a free lightweight, cross-platform 2D and 3D game engine implemented in C++ and released under the MIT license. Greatly inspired by OGRE (http://www.ogre3d.org) and Horde3D (http://www.horde3d.org).")
execute_process (COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/CMake/Modules/GetUrhoRevision.cmake WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} OUTPUT_VARIABLE URHO3D_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
string (REGEX MATCH "([^.]+)\\.([^.]+)\\.(.+)" MATCHED "${URHO3D_VERSION}")

# Setup SDK install destinations
if (WIN32)
    set (SCRIPT_EXT .bat)
else ()
    set (SCRIPT_EXT .sh)
endif ()
if (ANDROID)
    # For Android platform, install to a path based on the chosen Android ABI, e.g. libs/armeabi-v7a
    set (LIB_SUFFIX s/${ANDROID_NDK_ABI_NAME})
endif ()
set (DEST_BASE_INCLUDE_DIR include)
set (DEST_INCLUDE_DIR ${DEST_BASE_INCLUDE_DIR}/Urho3D)
set (DEST_BIN_DIR bin)
set (DEST_BIN_DIR_CONFIG ${DEST_BIN_DIR})
set (DEST_TOOLS_DIR ${DEST_BIN_DIR})
set (DEST_SAMPLES_DIR ${DEST_BIN_DIR})
set (DEST_SHARE_DIR share)
set (DEST_RESOURCE_DIR ${DEST_BIN_DIR})
set (DEST_BUNDLE_DIR ${DEST_SHARE_DIR}/Applications)
set (DEST_ARCHIVE_DIR lib)
set (DEST_PKGCONFIG_DIR ${DEST_ARCHIVE_DIR}/pkgconfig)
set (DEST_THIRDPARTY_HEADERS_DIR ${DEST_INCLUDE_DIR}/ThirdParty)
if (ANDROID)
    set (DEST_LIBRARY_DIR ${DEST_ARCHIVE_DIR})
else ()
    set (DEST_LIBRARY_DIR bin)
endif ()
set (DEST_LIBRARY_DIR_CONFIG ${DEST_LIBRARY_DIR})
if (MSVC)
    set (DEST_BIN_DIR_CONFIG ${DEST_BIN_DIR_CONFIG}/$<CONFIG>)
    set (DEST_LIBRARY_DIR_CONFIG ${DEST_LIBRARY_DIR}/$<CONFIG>)
    ucm_add_flags(/W1)
endif ()
if (WIN32)
    set (WINVER 0x0601)
    add_definitions(-DWINVER=${WINVER} -D_WIN32_WINNT=${WINVER} -D_WIN32_WINDOWS=${WINVER})
endif ()

set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${DEST_BIN_DIR})
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${DEST_LIBRARY_DIR})
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${DEST_ARCHIVE_DIR})

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -DURHO3D_DEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DURHO3D_DEBUG")
if (NOT DEFINED URHO3D_64BIT)
    if (CMAKE_SIZEOF_VOID_P MATCHES 8)
        set(URHO3D_64BIT ON)
    else ()
        set(URHO3D_64BIT OFF)
    endif ()
endif ()

if (MINGW)
    find_file(DLL_FILE_PATH_1 "libstdc++-6.dll")
    find_file(DLL_FILE_PATH_2 "libgcc_s_seh-1.dll")
    find_file(DLL_FILE_PATH_3 "libwinpthread-1.dll")
    foreach (DLL_FILE_PATH ${DLL_FILE_PATH_1} ${DLL_FILE_PATH_2} ${DLL_FILE_PATH_3})
        if (DLL_FILE_PATH)
            # Copies dlls to bin or tools.
            file (COPY ${DLL_FILE_PATH} DESTINATION ${CMAKE_BINARY_DIR}/${DEST_TOOLS_DIR})
            if (NOT URHO3D_STATIC_RUNTIME)
                file (COPY ${DLL_FILE_PATH} DESTINATION ${CMAKE_BINARY_DIR}/${DEST_SAMPLES_DIR})
            endif ()
        endif ()
    endforeach ()
endif ()

# Configure for web
if (EMSCRIPTEN)
    # Emscripten-specific setup
    if (EMSCRIPTEN_EMCC_VERSION VERSION_LESS 1.31.3)
        message(FATAL_ERROR "Unsupported compiler version")
    endif ()
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-warn-absolute-paths -Wno-unknown-warning-option")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-warn-absolute-paths -Wno-unknown-warning-option")
    if (URHO3D_THREADING)
        set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -s USE_PTHREADS=1")
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s USE_PTHREADS=1")
    endif ()
    set (CMAKE_C_FLAGS_RELEASE "-Oz -DNDEBUG")
    set (CMAKE_CXX_FLAGS_RELEASE "-Oz -DNDEBUG")
    # Remove variables to make the -O3 regalloc easier, embed data in asm.js to reduce number of moving part
    set (CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -O3 -s AGGRESSIVE_VARIABLE_ELIMINATION=1 --memory-init-file 0")
    set (CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} -O3 -s AGGRESSIVE_VARIABLE_ELIMINATION=1 --memory-init-file 0")
    # Preserve LLVM debug information, show line number debug comments, and generate source maps; always disable exception handling codegen
    set (CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -g4 -s DISABLE_EXCEPTION_CATCHING=1")
    set (CMAKE_MODULE_LINKER_FLAGS_DEBUG "${CMAKE_MODULE_LINKER_FLAGS_DEBUG} -g4 -s DISABLE_EXCEPTION_CATCHING=1")
endif ()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set (CLANG ON)
    set (GNU ON)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set (GCC ON)
    set (GNU ON)
endif()

if ("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Linux")
    set (HOST_LINUX 1)
elseif ("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Windows")
    set (HOST_WIN32 1)
elseif ("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Darwin")
    set (HOST_MACOS 1)
endif ()

# Macro for setting symbolic link on platform that supports it
macro (create_symlink SOURCE DESTINATION)
    # Make absolute paths so they work more reliably on cmake-gui
    if (IS_ABSOLUTE ${SOURCE})
        set (ABS_SOURCE ${SOURCE})
    else ()
        set (ABS_SOURCE ${CMAKE_SOURCE_DIR}/${SOURCE})
    endif ()
    if (IS_ABSOLUTE ${DESTINATION})
        set (ABS_DESTINATION ${DESTINATION})
    else ()
        set (ABS_DESTINATION ${CMAKE_BINARY_DIR}/${DESTINATION})
    endif ()
    if (CMAKE_HOST_WIN32)
        if (IS_DIRECTORY ${ABS_SOURCE})
            set (SLASH_D /D)
        else ()
            unset (SLASH_D)
        endif ()
        set (RESULT_CODE 1)
        if(${CMAKE_SYSTEM_VERSION} GREATER_EQUAL 6.0)
            if (NOT EXISTS ${ABS_DESTINATION})
                # Have to use string-REPLACE as file-TO_NATIVE_PATH does not work as expected with MinGW on "backward slash" host system
                string (REPLACE / \\ BACKWARD_ABS_DESTINATION ${ABS_DESTINATION})
                string (REPLACE / \\ BACKWARD_ABS_SOURCE ${ABS_SOURCE})
                execute_process (COMMAND cmd /C mklink ${SLASH_D} ${BACKWARD_ABS_DESTINATION} ${BACKWARD_ABS_SOURCE} OUTPUT_QUIET ERROR_QUIET RESULT_VARIABLE RESULT_CODE)
            endif ()
        endif ()
        if (NOT "${RESULT_CODE}" STREQUAL "0")
            if (SLASH_D)
                set (COMMAND COMMAND ${CMAKE_COMMAND} -E copy_directory ${ABS_SOURCE} ${ABS_DESTINATION})
            else ()
                set (COMMAND COMMAND ${CMAKE_COMMAND} -E copy_if_different ${ABS_SOURCE} ${ABS_DESTINATION})
            endif ()
            # Fallback to copy only one time
            if (NOT EXISTS ${ABS_DESTINATION})
                execute_process (${COMMAND})
            endif ()
        endif ()
    else ()
        execute_process (COMMAND ${CMAKE_COMMAND} -E create_symlink ${ABS_SOURCE} ${ABS_DESTINATION})
    endif ()
endmacro ()

macro (add_sample TARGET)
    if ("${ARGN}" STREQUAL CSHARP)
        add_target_csharp(${TARGET} ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET}.csproj Urho3DNet)
        #install (FILES ${NET_OUTPUT_DIRECTORY}/${BIND_MANAGED_TARGET}.dll DESTINATION ${DEST_LIBRARY_DIR})
    else ()
        file (GLOB SOURCE_FILES *.cpp *.h)
        if (NOT URHO3D_WIN32_CONSOLE)
            set (TARGET_TYPE WIN32)
        endif ()
        if (ANDROID)
            add_library(${TARGET} SHARED ${SOURCE_FILES})
        else ()
            add_executable (${TARGET} ${TARGET_TYPE} ${SOURCE_FILES})
        endif ()
        target_link_libraries (${TARGET} Urho3D)
        target_include_directories(${TARGET} PRIVATE ..)
        if (NOT ANDROID)
            install(TARGETS ${TARGET} RUNTIME DESTINATION ${DEST_SAMPLES_DIR})
        endif ()

        if (EMSCRIPTEN)
            add_dependencies(${TARGET} pkg_resources_web)
            set_target_properties (${TARGET} PROPERTIES SUFFIX .html)
            if (EMSCRIPTEN_MEMORY_LIMIT)
                math(EXPR EMSCRIPTEN_TOTAL_MEMORY "${EMSCRIPTEN_MEMORY_LIMIT} * 1024 * 1024")
                target_link_libraries(${TARGET} "-s TOTAL_MEMORY=${EMSCRIPTEN_TOTAL_MEMORY}")
            endif ()
            if (EMSCRIPTEN_MEMORY_GROWTH)
                target_link_libraries(${TARGET} "-s ALLOW_MEMORY_GROWTH=1")
            endif ()
            target_link_libraries(${TARGET} "-s NO_EXIT_RUNTIME=1" "-s WASM=1" "--pre-js ${CMAKE_BINARY_DIR}/Source/pak-loader.js")
        endif ()
    endif ()
endmacro ()

# Groups sources into subfolders.
macro(group_sources)
    file (GLOB_RECURSE children LIST_DIRECTORIES true RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/**)
    foreach (child ${children})
        if (IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${child})
            string(REPLACE "/" "\\" groupname "${child}")
            file (GLOB files LIST_DIRECTORIES false ${CMAKE_CURRENT_SOURCE_DIR}/${child}/*)
            source_group(${groupname} FILES ${files})
        endif ()
    endforeach ()
endmacro()

macro (initialize_submodule SUBMODULE_DIR)
    file(GLOB SUBMODULE_FILES ${SUBMODULE_DIR}/*)
    list(LENGTH SUBMODULE_FILES SUBMODULE_FILES_LEN)

    if (SUBMODULE_FILES_LEN LESS 2)
        find_program(GIT git)
        if (NOT GIT)
            message(FATAL_ERROR "git not found.")
        endif ()
        execute_process(
            COMMAND git submodule update --init --remote "${SUBMODULE_DIR}"
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            RESULT_VARIABLE SUBMODULE_RESULT
        )
        if (NOT SUBMODULE_RESULT EQUAL 0)
            message(FATAL_ERROR "Failed to initialize submodule ${SUBMODULE_DIR}")
        endif ()
    endif ()
endmacro ()

function(vs_group_subdirectory_targets DIR FOLDER_NAME)
    get_property(DIRS DIRECTORY ${DIR} PROPERTY SUBDIRECTORIES)
    foreach(DIR ${DIRS})
        get_property(TARGETS DIRECTORY ${DIR} PROPERTY BUILDSYSTEM_TARGETS)
        foreach(TARGET ${TARGETS})
            get_target_property(TARGET_TYPE ${TARGET} TYPE)
            if (NOT ${TARGET_TYPE} STREQUAL "INTERFACE_LIBRARY")
                set_target_properties(${TARGET} PROPERTIES FOLDER ${FOLDER_NAME})
            endif ()
        endforeach()
        vs_group_subdirectory_targets(${DIR} ${FOLDER_NAME})
    endforeach()
endfunction()

if (URHO3D_CSHARP)
    if (NOT SWIG_EXECUTABLE)
        # Prebuilt files are mainly for windows/CI. If you run linux or macos you should probably build SWIG yourself.
        # A SWIG distribution built from code at https://github.com/rokups/swig/tree/Urho3D
        # You may build it yourself and set SWIG_EXECUTABLE in order to not use prebuilt binaries.
        file(DOWNLOAD https://github.com/rokups/Urho3D/files/2343051/swig-dist.zip ${CMAKE_BINARY_DIR}/swig-dist.zip)
        execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf swig-dist.zip WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
        if (HOST_LINUX)
            set (SWIG_PLATFORM lin64)
            set (SWIG_SHARE share/swig)
        elseif (HOST_WIN32)
            set (SWIG_PLATFORM win64)
            set (SWIG_SHARE share)
        elseif (HOST_MACOS)
            set (SWIG_PLATFORM mac64)
            set (SWIG_SHARE share/swig)
        endif ()
        set (SWIG_EXECUTABLE ${CMAKE_BINARY_DIR}/swig-dist/${SWIG_PLATFORM}/bin/swig${CMAKE_EXECUTABLE_SUFFIX})
        # Get SWIG version
        execute_process(COMMAND ${SWIG_EXECUTABLE} -version OUTPUT_VARIABLE SWIG_VERSION)
        string(REGEX MATCH "SWIG Version ([0-9\\.]+)" SWIG_VERSION "${SWIG_VERSION}")
        string(REPLACE "SWIG Version " "" SWIG_VERSION "${SWIG_VERSION}")
        string(STRIP "${SWIG_VERSION}" SWIG_VERSION)
        # Swig library path is embeded into executable. We extracted swig somewhere else other than prefix path so we
        # must override library path here.
        set (SWIG_DIR ${CMAKE_BINARY_DIR}/swig-dist/${SWIG_PLATFORM}/${SWIG_SHARE}/${SWIG_VERSION})
        message(STATUS "Using prebuilt SWIG binaries for ${SWIG_PLATFORM} platform.")
    endif ()
    include(UrhoSWIG)

    if (WIN32)
        find_package(Mono REQUIRED)
    endif ()
    find_program(MSBUILD msbuild PATHS /Library/Frameworks/Mono.framework/Versions/Current/bin ${MONO_PATH}/bin)
    if (NOT MSBUILD)
        message(FATAL_ERROR "MSBuild could not be found.")
    endif ()

    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        set (CSHARP_PLATFORM x64)
    else ()
        set (CSHARP_PLATFORM x86)
    endif ()
    set (MSBUILD_COMMON_PARAMETERS /p:BuildDir="${CMAKE_BINARY_DIR}/" /p:Platform=${CSHARP_PLATFORM}
                                   /p:Configuration=$<CONFIG> /consoleloggerparameters:ErrorsOnly)

    if (URHO3D_WITH_MONO)
        list (APPEND MSBUILD_COMMON_PARAMETERS /p:TargetFramework=net471)
    endif ()

    if (MSVC)
        set (CSHARP_SOLUTION ${CMAKE_BINARY_DIR}/Urho3D.sln)
    else ()
        set (CSHARP_SOLUTION ${Urho3D_SOURCE_DIR}/Urho3D.part.sln)
    endif ()

    add_custom_target(NugetRestore COMMAND ${TERM_WORKAROUND} ${MSBUILD} ${CSHARP_SOLUTION} /t:restore)
endif()

if ("${CMAKE_HOST_SYSTEM_NAME}" STREQUAL "Linux")
    # Workaround for some cases where csc has issues when invoked by CMake.
    set (TERM_WORKAROUND env TERM=xterm)
endif ()

macro (__TARGET_GET_PROPERTIES_RECURSIVE OUTPUT TARGET PROPERTY)
    get_target_property(values ${TARGET} ${PROPERTY})
    if (values)
        list (APPEND ${OUTPUT} ${values})
    endif ()
    get_target_property(values ${TARGET} INTERFACE_LINK_LIBRARIES)
    if (values)
        foreach(lib ${values})
            if (TARGET ${lib})
                __TARGET_GET_PROPERTIES_RECURSIVE(${OUTPUT} ${lib} ${PROPERTY})
            endif ()
        endforeach()
    endif()
endmacro()

macro (add_target_csharp TARGET PROJECT_FILE)
    if (WIN32 AND NOT URHO3D_WITH_MONO)
        include_external_msproject(${TARGET} ${PROJECT_FILE} TYPE FAE04EC0-301F-11D3-BF4B-00C04F79EFBC ${ARGN} NugetRestore)
    else ()
        add_custom_target(${TARGET}
            COMMAND ${TERM_WORKAROUND} ${MSBUILD} ${PROJECT_FILE} ${MSBUILD_COMMON_PARAMETERS}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            DEPENDS ${ARGN} NugetRestore
        )
        set_target_properties(${TARGET} PROPERTIES EXCLUDE_FROM_ALL OFF)
    endif ()
endmacro ()

macro (csharp_bind_target)
    if (NOT URHO3D_CSHARP)
        return ()
    endif ()

    cmake_parse_arguments(BIND "" "TARGET;MANAGED_TARGET" "" ${ARGN})

    get_target_property(BIND_SOURCE_DIR ${BIND_TARGET} SOURCE_DIR)

    if (NOT BIND_MANAGED_TARGET)
        set (BIND_MANAGED_TARGET ${BIND_TARGET}Net)
    endif ()

    # Parse bindings using same compile definitions as built target
    __TARGET_GET_PROPERTIES_RECURSIVE(INCLUDES ${BIND_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
    __TARGET_GET_PROPERTIES_RECURSIVE(DEFINES  ${BIND_TARGET} INTERFACE_COMPILE_DEFINITIONS)
    __TARGET_GET_PROPERTIES_RECURSIVE(OPTIONS  ${BIND_TARGET} INTERFACE_COMPILE_OPTIONS)
    if (INCLUDES)
        list (REMOVE_DUPLICATES INCLUDES)
    endif ()
    if (DEFINES)
        list (REMOVE_DUPLICATES DEFINES)
    endif ()
    if (OPTIONS)
        list (REMOVE_DUPLICATES OPTIONS)
    endif ()
    foreach(item ${INCLUDES})
        if ("${item}" MATCHES "\\$<INSTALL_INTERFACE:.+")
            continue()
        endif ()
        if ("${item}" MATCHES "\\$<BUILD_INTERFACE:.+")
            string(LENGTH "${item}" len)
            math(EXPR len "${len} - 19")
            string(SUBSTRING "${item}" 18 ${len} item)
        endif ()
        list(APPEND GENERATOR_OPTIONS -I${item})
    endforeach()
    foreach(item ${DEFINES})
        string(FIND "${item}" "=" EQUALITY_INDEX)
        if (EQUALITY_INDEX EQUAL -1)
            set (item "${item}=1")
        endif ()
        list(APPEND GENERATOR_OPTIONS -D${item})
    endforeach()

    # Swig
    set(CMAKE_SWIG_FLAGS
        -namespace ${BIND_MANAGED_TARGET}
        -fastdispatch
        -I${CMAKE_CURRENT_BINARY_DIR}
        ${GENERATOR_OPTIONS}
    )

    foreach(item ${OPTIONS})
        list(APPEND GENERATOR_OPTIONS -O${item})
    endforeach()

    #    get_target_property(_TARGET_TYPE ${BIND_TARGET} TYPE)
    #    if(_TARGET_TYPE STREQUAL "STATIC_LIBRARY")
    #        list (APPEND GENERATOR_OPTIONS --static)
    #    endif ()
    #    if (SNK_PUB_KEY)
    #        list (APPEND GENERATOR_OPTIONS --signed-with=${SNK_PUB_KEY})
    #    endif ()

    set (CSHARP_LIBRARY_NAME ${BIND_TARGET}CSharp)
    set (COMMON_DIR ${CSHARP_SOURCE_DIR}/Common)

    # Finalize option list
    list (APPEND GENERATOR_OPTIONS ${BIND_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/Swig)
    set (CSHARP_BINDING_GENERATOR_OPTIONS "${CMAKE_CURRENT_BINARY_DIR}/generator_options.txt")
    file (WRITE ${CSHARP_BINDING_GENERATOR_OPTIONS} "")
    foreach (opt ${GENERATOR_OPTIONS})
        file(APPEND ${CSHARP_BINDING_GENERATOR_OPTIONS} "${opt}\n")
    endforeach ()

    set (SWIG_SYSTEM_INCLUDE_DIRS "${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES};${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES};${CMAKE_SYSTEM_INCLUDE_PATH};${CMAKE_EXTRA_GENERATOR_CXX_SYSTEM_INCLUDE_DIRS}")
    string (REPLACE ";" ";-I" SWIG_SYSTEM_INCLUDE_DIRS "${SWIG_SYSTEM_INCLUDE_DIRS}")

    set_source_files_properties(Swig/${BIND_TARGET}.i PROPERTIES
        CPLUSPLUS ON
        SWIG_FLAGS "-I${SWIG_SYSTEM_INCLUDE_DIRS}"
    )

    swig_add_module(${CSHARP_LIBRARY_NAME} csharp Swig/${BIND_TARGET}.i)
    swig_link_libraries(${CSHARP_LIBRARY_NAME} ${BIND_TARGET})
    set_target_properties(${CSHARP_LIBRARY_NAME} PROPERTIES PREFIX "${CMAKE_SHARED_LIBRARY_PREFIX}")

    # Etc
    #    if (CLANG)
    #        target_compile_options(${CSHARP_LIBRARY_NAME} PRIVATE -Wno-return-type-c-linkage)
    #    endif ()
    #
    #    if (URHO3D_WITH_MONO)
    #        find_package(Mono REQUIRED)
    #        target_include_directories(${CSHARP_LIBRARY_NAME} PRIVATE ${MONO_INCLUDE_DIRS})
    #        target_link_libraries(${CSHARP_LIBRARY_NAME} ${MONO_LIBRARIES})
    #        target_compile_options(${CSHARP_LIBRARY_NAME} PRIVATE ${MONO_CFLAGS})
    #    endif ()

    if (MSVC)
        set (NET_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIG>)
    else ()
        set (NET_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
        # Needed for mono on unixes but not on windows.
        set (FACADES Facades/)
    endif ()
    if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${BIND_MANAGED_TARGET}.csproj")
        add_target_csharp(${BIND_MANAGED_TARGET} ${CMAKE_CURRENT_SOURCE_DIR}/${BIND_MANAGED_TARGET}.csproj ${CSHARP_LIBRARY_NAME})
        install (FILES ${NET_OUTPUT_DIRECTORY}/${BIND_MANAGED_TARGET}.dll DESTINATION ${DEST_LIBRARY_DIR})
    endif ()

    file (GLOB_RECURSE EXTRA_NATIVE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/Native/*.h ${CMAKE_CURRENT_SOURCE_DIR}/Native/*.cpp)
    target_sources(${CSHARP_LIBRARY_NAME} PRIVATE ${EXTRA_NATIVE_FILES})
endmacro ()

# Configure for MingW
if (CMAKE_CROSSCOMPILING AND MINGW)
    # Symlink windows libraries and headers to appease some libraries that do not use all-lowercase names and break on
    # case-sensitive file systems.
    file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/workaround)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/include/windows.h workaround/Windows.h)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/include/shobjidl.h workaround/ShObjIdl.h)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/include/strsafe.h workaround/Strsafe.h)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/include/psapi.h workaround/Psapi.h)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/include/sddl.h workaround/Sddl.h)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/include/accctrl.h workaround/AccCtrl.h)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/include/aclapi.h workaround/Aclapi.h)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/include/oleidl.h workaround/OleIdl.h)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/include/shlobj.h workaround/Shlobj.h)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/lib/libws2_32.a workaround/libWs2_32.a)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/lib/libiphlpapi.a workaround/libIphlpapi.a)
    create_symlink(/usr/${CMAKE_SYSTEM_PROCESSOR}-w64-mingw32/lib/libwldap32.a workaround/libWldap32.a)
    include_directories(${CMAKE_BINARY_DIR}/workaround)
    link_libraries(-L${CMAKE_BINARY_DIR}/workaround)
endif ()
