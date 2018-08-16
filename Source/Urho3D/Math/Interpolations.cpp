#include "Interpolations.h"
#include "Vector3.h"

namespace Urho3D {

    float BilinearInterpolation(Vector3 topLeft, Vector3 topRight, Vector3 bottomLeft, Vector3 bottomRight, Vector2 samplePoint)
    {
        float x_y1 = ((bottomRight.x_ - samplePoint.x_) / (bottomRight.x_ - bottomLeft.x_))*(bottomLeft.z_) + ((samplePoint.x_ - bottomLeft.x_) / (bottomRight.x_ - bottomLeft.x_))*(bottomRight.z_);
        float x_y2 = ((bottomRight.x_ - samplePoint.x_) / (bottomRight.x_ - bottomLeft.x_))*(topLeft.z_) + ((samplePoint.x_ - bottomLeft.x_) / (bottomRight.x_ - bottomLeft.x_))*(topRight.z_);

        return ((topLeft.y_ - samplePoint.y_) / (topLeft.y_ - bottomLeft.y_))*(x_y1)+((samplePoint.y_ - bottomLeft.y_) / (topLeft.y_ - bottomLeft.y_))*(x_y2);
    }


}
