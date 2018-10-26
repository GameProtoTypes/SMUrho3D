
add_library(mlpack_wrap STATIC dummy.cpp)

#find_library(MLPACK_HOME mlpack)

find_path (MLPACK_HOME mlpack)
find_path (ARMADILLO_INCLUDE mlpack)


message(${MLPACK_HOME}/include)
target_include_directories(mlpack_wrap PUBLIC ${MLPACK_HOME}/include)
target_include_directories(mlpack_wrap PUBLIC ${ARMADILLO_INCLUDE})


target_link_libraries(mlpack_wrap "${MLPACK_HOME}/Release/mlpack.lib")


#install (TARGETS mlpack_wrap EXPORT Urho3D)
#install (DIRECTORY include/ DESTINATION ${DEST_THIRDPARTY_HEADERS_DIR}/ FILES_MATCHING PATTERN *.h)

#target_include_directories(cr INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)


install (TARGETS mlpack_wrap EXPORT Urho3D
    LIBRARY DESTINATION ${DEST_LIBRARY_DIR}
    RUNTIME DESTINATION ${DEST_BIN_DIR}
    ARCHIVE DESTINATION ${DEST_ARCHIVE_DIR}
)