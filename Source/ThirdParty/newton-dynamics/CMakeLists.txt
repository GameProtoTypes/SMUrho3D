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



# Define source files
file (GLOB_RECURSE SOURCE_FILES *.h *.cpp)
set (SOURCE_FILES ${SOURCE_FILES})
set (LIB_TYPE STATIC)


add_library (newton-dynamics ${LIB_TYPE} ${SOURCE_FILES})


target_compile_definitions(newton-dynamics PUBLIC _CNEWTON_STATIC_LIB _CUSTOM_JOINTS_STATIC_LIB _NEWTON_STATIC_LIB)


target_include_directories(newton-dynamics
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/dAnimation>  
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/dContainers>  
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/dCustomJoints>  
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/dgCore>  
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/dgMeshUtil>  
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/dgNewton>  
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/dgPhysics>  
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/dgNewtonCpu> 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/dMath>  
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/dNewton>  
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/dScene>  
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sdk/thirdParty/tinyxml>  
        $<INSTALL_INTERFACE:${DEST_THIRDPARTY_HEADERS_DIR}/sdk>
)




install (TARGETS newton-dynamics EXPORT Urho3D ARCHIVE DESTINATION ${DEST_ARCHIVE_DIR})
install(DIRECTORY newton-dynamics DESTINATION ${DEST_THIRDPARTY_HEADERS_DIR} FILES_MATCHING PATTERN *.h)
