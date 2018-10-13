#pragma once

namespace Urho3D {
    class Vector3;
    class Vector2;

    //given 4 2d basis points with the z component of each specifying the value at the point, returns the interpolation value based on proximity of the samplePoint to the basis points.
    float BilinearInterpolation(Vector3 topLeft, Vector3 topRight, Vector3 bottomLeft, Vector3 bottomRight, Vector2 samplePoint);
}

