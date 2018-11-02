#!/bin/bash

rm -r -f build

cmake -Bbuild -H. \
-DURHO3D_RENDERER=OpenGL \
-DURHO3D_ENABLE_ALL=ON \
-DBUILD_SHARED_LIBS=ON \
-DURHO3D_CSHARP=OFF \
