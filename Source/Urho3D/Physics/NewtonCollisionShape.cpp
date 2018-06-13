
#include "NewtonCollisionShape.h"
#include "../Core/Context.h"
#include "Newton.h"
#include "NewtonPhysicsWorld.h"
#include "Scene/Component.h"
#include "Scene/Node.h"
#include "Scene/Scene.h"
#include "dMatrix.h"
namespace Urho3D {



    NewtonCollisionShape::NewtonCollisionShape(Context* context) : Component(context)
    {

    }

    NewtonCollisionShape::~NewtonCollisionShape()
    {

    }

    void NewtonCollisionShape::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonCollisionShape>();
    }

    void NewtonCollisionShape::SetBox(const Vector3& size, const Vector3& position, const Quaternion& rotation)
    {
        NewtonWorld* world = GetScene()->GetComponent<NewtonPhysicsWorld>()->GetNewtonWorld();

        Matrix4 mat;
        mat.SetScale(size);
        mat.SetTranslation(position);
        mat.SetRotation(rotation.RotationMatrix());
        dMatrix nMat = UrhoToNewton(mat);


        newtonCollision_= NewtonCreateBox(world, 1.0f, 1.0f, 1.0f, 0, &nMat[0][0]);
        NewtonCollisionSetUserData(newtonCollision_, (void*)this);
    }


}
