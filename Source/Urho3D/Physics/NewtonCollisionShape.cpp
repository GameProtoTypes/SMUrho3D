
#include "NewtonCollisionShape.h"
#include "../Core/Context.h"
#include "Newton.h"
#include "NewtonPhysicsWorld.h"
#include "Scene/Component.h"
#include "Scene/Node.h"
#include "Scene/Scene.h"
#include "dMatrix.h"
#include "IO/Log.h"
namespace Urho3D {



    NewtonCollisionShape::NewtonCollisionShape(Context* context) : Component(context)
    {

    }

    NewtonCollisionShape::~NewtonCollisionShape()
    {
        freeInternalCollision();
    }

    void NewtonCollisionShape::RegisterObject(Context* context)
    {
        bool s = context->RegisterFactory<NewtonCollisionShape>();
    }

    void NewtonCollisionShape::SetBox(const Vector3& size, const Vector3& position, const Quaternion& rotation)
    {
        NewtonWorld* world = GetScene()->GetComponent<NewtonPhysicsWorld>()->GetNewtonWorld();

        Matrix4 mat;
        mat.SetScale(size);
        mat.SetTranslation(position);
        mat.SetRotation(rotation.RotationMatrix());
        dMatrix nMat = UrhoToNewton(mat);

        newtonCollision_ = NewtonCreateBox(world, 1.0f, 1.0f, 1.0f, 0, &nMat[0][0]);
        NewtonCollisionSetUserData(newtonCollision_, (void*)this);
    }


    NewtonCollision* NewtonCollisionShape::GetNewtonCollision()
    {
        return newtonCollision_;
    }

    void NewtonCollisionShape::freeInternalCollision()
    {
        if (newtonCollision_) {
            NewtonDestroyCollision(newtonCollision_);
            newtonCollision_ = nullptr;
        }
    }

    void NewtonCollisionShape::addToPhysicsWorld()
    {
        if (physicsWorld_)
            physicsWorld_->addCollisionShape(this);
    }

    void NewtonCollisionShape::removeFromPhysicsWorld()
    {
        if(physicsWorld_)
            physicsWorld_->removeCollisionShape(this);
    }

    void NewtonCollisionShape::OnNodeSet(Node* node)
    {
       // throw std::logic_error("The method or operation is not implemented.");
    }

    void NewtonCollisionShape::OnSceneSet(Scene* scene)
    {
        if (scene)
        {
            if (scene == node_)
                URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

            physicsWorld_ = WeakPtr<NewtonPhysicsWorld>(scene->GetOrCreateComponent<NewtonPhysicsWorld>());
            addToPhysicsWorld();

            // Create shape now if necessary (attributes modified before adding to scene)
            //if (retryCreation_)
            //{
            //    UpdateShape();
            //    NotifyRigidBody();
            //}
        }
        else
        {

            if (physicsWorld_)
                removeFromPhysicsWorld();

            freeInternalCollision();

            // Recreate when moved to a scene again
            //retryCreation_ = true;
        }
    }

    void NewtonCollisionShape::OnMarkedDirty(Node* node)
    {
       // throw std::logic_error("The method or operation is not implemented.");
    }

}
