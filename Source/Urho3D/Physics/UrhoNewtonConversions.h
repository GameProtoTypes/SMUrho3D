#pragma once


class dMatrix;
class dVector;

namespace Urho3D {
    class Matrix4;
    class Matrix3x4;
    class Vector3;
    class Vector4;


    ///Conversion Functions From Urho To Newton
    dMatrix UrhoToNewton(Matrix4 mat);
    dMatrix UrhoToNewton(Matrix3x4 mat);

    ///Conversion Function From Newton To Urho
    Vector3 NewtonToUrhoVec3(dVector vec);
    Vector4 NewtonToUrhoVec4(dVector vec);
    Matrix4 NewtonToUrhoMat4(dMatrix mat);

    ///Printing Helpers
    void PrintNewton(dMatrix mat);

}
