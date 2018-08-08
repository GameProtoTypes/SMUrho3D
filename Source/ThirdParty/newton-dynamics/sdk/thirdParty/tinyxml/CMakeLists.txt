# Copyright (c) <2014-2017> <Newton Game Dynamics>
#
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
#
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely.

cmake_minimum_required(VERSION 3.12.0)

set (projectName "tinyxml")
message (${projectName})


 # low level core
file(GLOB source *.cpp *.h)

add_library(${projectName} STATIC ${source})

#file(GLOB includes *.h)
#install(FILES ${includes} DESTINATION "${includePath}/${projectName}")

install(TARGETS ${projectName} EXPORT Urho3D ARCHIVE DESTINATION ${libraryPath})
install(DIRECTORY ${projectName} DESTINATION ${DEST_THIRDPARTY_HEADERS_DIR} FILES_MATCHING PATTERN *.h) 