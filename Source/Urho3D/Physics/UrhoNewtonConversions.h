#pragma once


class dMatrix;
class dVector;
class dgQuaternion;

namespace Urho3D {
    class Matrix4;
    class Matrix3x4;
    class Vector2;
    class Vector3;
    class Vector4;
    class Quaternion;

    ///Conversion Functions From Urho To Newton
    dMatrix UrhoToNewton(Matrix4 mat);
    dMatrix UrhoToNewton(Matrix3x4 mat);
    dVector UrhoToNewton(Vector3 vec4);
    dVector UrhoToNewton(Vector3 vec3);
    dVector UrhoToNewton(Vector2 vec2);


    ///Conversion Function From Newton To Urho
    Vector3 NewtonToUrhoVec3(dVector vec);
    Vector4 NewtonToUrhoVec4(dVector vec);
    Matrix4 NewtonToUrhoMat4(dMatrix mat);
    Quaternion NewtonToUrhoQuat(dgQuaternion quat);


    ///Printing Helpers
    void PrintNewton(dMatrix mat);

}
