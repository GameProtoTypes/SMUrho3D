
#include "NewtonCollisionShape.h"
#include "NewtonPhysicsWorld.h"
#include "NewtonRigidBody.h"

#include "../Core/Context.h"
#include "../Scene/Component.h"
#include "../Scene/Node.h"
#include "../Scene/Scene.h"
#include "../Graphics/Model.h"
#include "../IO/Log.h"

#include "Newton.h"
#include "dMatrix.h"

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
        shapeType_ = SHAPE_BOX;
        size_ = size;
        position_ = position;
        rotation_ = rotation;

        reEvaluateCollision();
        notifyRigidBody();
    }


    void NewtonCollisionShape::SetTriangleMesh(Model* model, unsigned lodLevel /*= 0*/, const Vector3& scale /*= Vector3::ONE*/, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    {
        if (!model)
            return;

        shapeType_ = SHAPE_TRIANGLEMESH;
        size_ = scale;
        position_ = position;
        rotation_ = rotation;
        model_ = model;
        modelLodLevel_ = lodLevel;



        reEvaluateCollision();
        notifyRigidBody();


    }

    void NewtonCollisionShape::reEvaluateCollision()
{
        // first free any reference to an existing collision.
        freeInternalCollision();

        NewtonWorld* world = GetScene()->GetComponent<NewtonPhysicsWorld>()->GetNewtonWorld();

        Matrix4 mat;
        //mat.SetScale(size_);
        mat.SetTranslation(position_);
        mat.SetRotation(rotation_.RotationMatrix());
        dMatrix nMat = UrhoToNewton(mat);


        if (shapeType_ == SHAPE_BOX) {

            // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily unique)
            newtonCollision_ = NewtonCreateBox(world, size_.x_*node_->GetScale().x_,
                size_.y_*node_->GetScale().y_,
                size_.z_*node_->GetScale().z_, 0, &nMat[0][0]);

        }
        ///.....
        else if (shapeType_ == SHAPE_TRIANGLEMESH)
        {
            dVector minBox = UrhoToNewton(model_->GetBoundingBox().min_);
            dVector maxBox = UrhoToNewton(model_->GetBoundingBox().max_);


            //newtonCollision_ = NewtonCreateUserMeshCollision(world, &minBox[0], &maxBox[0], nullptr, )
        }
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
            rigidBody_->reEvaluateBody();
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
       
    }

    void NewtonCollisionShape::OnSceneSet(Scene* scene)
    {
        if (scene)
        {
            if (scene == node_)
                URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

            physicsWorld_ = WeakPtr<NewtonPhysicsWorld>(scene->GetOrCreateComponent<NewtonPhysicsWorld>());

            reEvaluateCollision();


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
