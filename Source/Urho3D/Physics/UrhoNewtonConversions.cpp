#include "UrhoNewtonConversions.h"
#include "dMatrix.h"
#include "dVector.h"
#include "Math/Matrix4.h"
#include "Math/Matrix3x4.h"
#include "IO/Log.h"


namespace Urho3D {


    dMatrix UrhoToNewton(Matrix4 mat4)
    {
        return dMatrix(mat4.Transpose().Data());
    }
    dMatrix UrhoToNewton(Matrix3x4 mat3x4)
    {
        Matrix4 asMat4 = mat3x4.ToMatrix4();
        return dMatrix(asMat4.Transpose().Data());
    }


    Vector3 NewtonToUrhoVec3(dVector vec)
    {
        return Vector3(vec.m_x, vec.m_y, vec.m_z);
    }
    Vector4 NewtonToUrhoVec4(dVector vec)
    {
        return Vector4(vec.m_x, vec.m_y, vec.m_z, vec.m_w);
    }


    Matrix4 NewtonToUrhoMat4(dMatrix mat)
    {
        return Matrix4(&mat[0][0]).Transpose();
    }

    void PrintNewton(dMatrix mat)
    {
        for (int x = 0; x < 4; x++)
            URHO3D_LOGINFO(String(mat[x][0]) + " , " + String(mat[x][1]) + " , " + String(mat[x][2]) + " , " + String(mat[x][3]));

        URHO3D_LOGINFO("");

    }

}
