#pragma once


#include "../Scene/Component.h"

namespace Urho3D
{

    class Component;

    class URHO3D_API NewtonCollisionShape : public Component
    {
        URHO3D_OBJECT(NewtonCollisionShape, Component);
    public:

        NewtonCollisionShape(Context* context);

        ~NewtonCollisionShape() override;

        static void RegisterObject(Context* context);



        /// Set as a box.
        void SetBox(const Vector3& size, const Vector3& position = Vector3::ZERO, const Quaternion& rotation = Quaternion::IDENTITY);



    };



}
