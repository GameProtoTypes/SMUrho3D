
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
        NewtonWorld* world = GetScene()->GetComponent<NewtonPhysicsWorld>()->GetNewtonWorld();

        Matrix4 mat;
        mat.SetScale(size*node_->GetWorldScale());
        mat.SetTranslation(position);
        mat.SetRotation(rotation.RotationMatrix());
        dMatrix nMat = UrhoToNewton(mat);

        newtonCollision_ = NewtonCreateBox(world, 1.0f, 1.0f, 1.0f, 0, &nMat[0][0]);
        NewtonCollisionSetUserData(newtonCollision_, (void*)this);

        rigidBody_ = node_->GetComponent<NewtonRigidBody>();
        if (rigidBody_) {
            rigidBody_->onCollisionUpdated();
        }

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


    void NewtonCollisionShape::onRigidBodyUpdated()
    {
        NewtonRigidBody* rigidBody = node_->GetComponent<NewtonRigidBody>();
        rigidBody_ = rigidBody;
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
            physicsWorld_->addCollisionShape(this);




            rigidBody_ = node_->GetComponent<NewtonRigidBody>();
            if (rigidBody_) {
                rigidBody_->onCollisionUpdated();
            }


        }
        else
        {

            if (physicsWorld_)
                physicsWorld_->removeCollisionShape(this);

            freeInternalCollision();

            rigidBody_ = node_->GetComponent<NewtonRigidBody>();

            if (rigidBody_) {
                rigidBody_->onCollisionUpdated();

            }


        }
    }

    void NewtonCollisionShape::OnMarkedDirty(Node* node)
    {
       // throw std::logic_error("The method or operation is not implemented.");
    }

}
