#include "NewtonConstraint.h"
#include "NewtonRigidBody.h"
#include "UrhoNewtonPhysicsWorld.h"
#include "Core/Context.h"
#include "Scene/Component.h"
#include "Graphics/DebugRenderer.h"

#include "Scene/Scene.h"
#include "dCustomFixDistance.h"
namespace Urho3D {
    NewtonConstraint::NewtonConstraint(Context* context) : Component(context)
    {

    }

    NewtonConstraint::~NewtonConstraint()
    {
    }

    void NewtonConstraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonConstraint>(DEF_PHYSICS_CATEGORY.CString());
    }

    void NewtonConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        if (!ownBody_ || !otherBody_)
            return;

        debug->AddLine(ownBody_->GetCenterOfMassPosition(), otherBody_->GetCenterOfMassPosition(), Color::BLUE, false);
    }

    void NewtonConstraint::SetDisableCollision(bool disable)
    {
        enableBodyCollision_ = !disable;
        MarkDirty();
    }

    void NewtonConstraint::SetOtherBody(NewtonRigidBody* body)
    {
        if (otherBody_ != body) {

            if (otherBody_ != nullptr)
                RemoveJointReferenceFromBody(otherBody_);//remove reference from old body


            otherBody_ = body;
            AddJointReferenceToBody(otherBody_);
            otherBody_->GetNode()->AddListener(this);
            MarkDirty();
        }
    }


    void NewtonConstraint::SetPosition(const Vector3& position)
    {
        position_ = position;
        MarkDirty();
    }


    void NewtonConstraint::SetRotation(const Quaternion& rotation)
    {
        rotation_ = rotation_;
        MarkDirty();
    }

    void NewtonConstraint::SetOtherPosition(const Vector3& position)
    {
        otherPosition_ = position_;
        MarkDirty();
    }


    void NewtonConstraint::SetOtherRotation(const Quaternion& rotation)
    {
        otherRotation_ = rotation;
        MarkDirty();
    }

    void NewtonConstraint::OnSetEnabled()
    {
        MarkDirty();
    }

    void NewtonConstraint::reEvalConstraint()
    {

        freeConstraint();

        if (!IsEnabledEffective()) {
            MarkDirty(false);
            return;
        }

        buildConstraint();
        NewtonJointSetCollisionState((NewtonJoint*)newtonJoint_, enableBodyCollision_);
        MarkDirty(false);
        
    }

    void NewtonConstraint::buildConstraint()
    {
        /// ovverride in derived classes.
    }


    void NewtonConstraint::freeConstraint()
    {
        if (newtonJoint_ != nullptr) {
            delete newtonJoint_;
            newtonJoint_ = nullptr;
        }
    }



    void NewtonConstraint::AddJointReferenceToBody(NewtonRigidBody* rigBody)
    {

        if (!rigBody->connectedConstraints_.Contains(this))
            rigBody->connectedConstraints_.Insert(this);

    }


    void NewtonConstraint::RemoveJointReferenceFromBody(NewtonRigidBody* rigBody)
    {

        if (rigBody->connectedConstraints_.Contains(this))
            rigBody->connectedConstraints_.Erase(this);

    }

    void NewtonConstraint::OnNodeSet(Node* node)
    {
        if (node)
        {
            //auto create physics world similar to rigid body.
            physicsWorld_ = node->GetScene()->GetOrCreateComponent<UrhoNewtonPhysicsWorld>();

            NewtonRigidBody* rigBody = node->GetComponent<NewtonRigidBody>();
            if (rigBody) {
                ownBody_ = rigBody;
            }
           
            if(physicsWorld_)
                physicsWorld_->addConstraint(this);

            AddJointReferenceToBody(ownBody_);

            node->AddListener(this);

        }
        else
        {
            RemoveJointReferenceFromBody(ownBody_);


            ownBody_ = nullptr;
            if (physicsWorld_)
                physicsWorld_->removeConstraint(this);

            freeConstraint();

        }
    }

    void NewtonConstraint::OnNodeSetEnabled(Node* node)
    {
        MarkDirty();
    }

}
