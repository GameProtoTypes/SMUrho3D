
#include "NewtonCollisionShape.h"
#include "../Core/Context.h"
#include "Newton.h"
#include "NewtonPhysicsWorld.h"
#include "NewtonRigidBody.h"
#include "Scene/Component.h"
#include "Scene/Node.h"
#include "Scene/Scene.h"
#include "dMatrix.h"
#include "IO/Log.h"
#include "UrhoNewtonConversions.h"
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
        size_ = size;
        position_ = position;
        rotation_ = rotation;

        resolveCollision();
        notifyRigidBody();
    }


    void NewtonCollisionShape::resolveCollision()
    {
        // first free any reference to an existing collision.
        freeInternalCollision();

        NewtonWorld* world = GetScene()->GetComponent<NewtonPhysicsWorld>()->GetNewtonWorld();

        Matrix4 mat;
        mat.SetScale(size_);
        mat.SetTranslation(position_);
        mat.SetRotation(rotation_.RotationMatrix());
        dMatrix nMat = UrhoToNewton(mat);
        
        // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily unique)
        newtonCollision_ = NewtonCreateBox(world, 1.0f, 1.0f, 1.0f, 0, &nMat[0][0]);
    }

    void NewtonCollisionShape::freeInternalCollision()
    {
        if (newtonCollision_) {
            NewtonDestroyCollision(newtonCollision_);//decrement the reference count of the collision.
            newtonCollision_ = nullptr;
        }
    }

    void NewtonCollisionShape::notifyRigidBody()
    {
        rigidBody_ = node_->GetComponent<NewtonRigidBody>();
        if (rigidBody_) {
            rigidBody_->rebuildBody();
        }
    }

    NewtonCollision* NewtonCollisionShape::GetNewtonCollision()
    {
        return newtonCollision_;
    }




    void NewtonCollisionShape::updateReferenceToRigidBody()
{
        rigidBody_ = node_->GetComponent<NewtonRigidBody>();
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

            resolveCollision();


            physicsWorld_->addCollisionShape(this);


            notifyRigidBody();
        }
        else
        {
            freeInternalCollision();

            if (physicsWorld_)
                physicsWorld_->removeCollisionShape(this);


            notifyRigidBody();
        }
    }

    void NewtonCollisionShape::OnMarkedDirty(Node* node)
    {
       // throw std::logic_error("The method or operation is not implemented.");
    }

}
