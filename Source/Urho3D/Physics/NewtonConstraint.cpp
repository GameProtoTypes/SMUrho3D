#include "NewtonConstraint.h"
#include "RigidBody.h"
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
        URHO3D_COPY_BASE_ATTRIBUTES(Component);

        URHO3D_ACCESSOR_ATTRIBUTE("Other Body ID", GetOtherBodyId, SetOtherBodyById, unsigned, 0, AM_DEFAULT | AM_COMPONENTID);
        URHO3D_ACCESSOR_ATTRIBUTE("Other Body Position", GetOtherPosition, SetOtherPosition, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Other Body Rotation", GetOtherRotation, SetOtherRotation, Quaternion, Quaternion::IDENTITY, AM_DEFAULT);


    }

    void NewtonConstraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        if (ownBody_ && otherBody_) {
            debug->AddLine(ownBody_->GetNode()->LocalToWorld(position_), otherBody_->GetNode()->LocalToWorld(otherPosition_), Color::BLUE, false);
            
        }
    }

    void NewtonConstraint::SetDisableCollision(bool disable)
    {
        enableBodyCollision_ = !disable;
        MarkDirty();
    }

    void NewtonConstraint::SetOtherBody(RigidBody* body)
    {
        if (otherBody_ != body) {

            if (otherBody_ != nullptr)
                RemoveJointReferenceFromBody(otherBody_);//remove reference from old body


            otherBody_ = body;
            if (body != nullptr) {
                AddJointReferenceToBody(body);
                body->GetNode()->AddListener(this);
            }

            if (body == nullptr)
                otherBodyId_ = 0;
            else
                otherBodyId_ = body->GetID();

            MarkDirty();
        }
    }


    void NewtonConstraint::SetOtherBodyById(unsigned bodyId)
    {
        otherBodyId_ = bodyId;
        //resolve to body later.
        MarkDirty();
    }

    void NewtonConstraint::SetPosition(const Vector3& position)
    {
        position_ = position;
        MarkDirty();
    }


    void NewtonConstraint::SetRotation(const Quaternion& rotation)
    {
        rotation_ = rotation;
        MarkDirty();
    }


    void NewtonConstraint::SetOtherPosition(const Vector3& position)
    {
        otherPosition_ = position;
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
        //resolve other body id to component
        otherBody_ = static_cast<RigidBody*>(GetScene()->GetComponent(otherBodyId_));


        if (!IsEnabledEffective()) {
            freeInternal();
        }
        else if (ownBody_ && ownBody_->GetNode() && ownBody_->GetNewtonBody()) {
            freeInternal();
            buildConstraint();
            NewtonJointSetCollisionState((NewtonJoint*)newtonJoint_, enableBodyCollision_);
        }
        else//we dont have own body so free the joint..
        {
            freeInternal();
        }
        MarkDirty(false);
    }

    void NewtonConstraint::buildConstraint()
    {
        /// ovverride in derived classes.
    }


    void NewtonConstraint::freeInternal()
    {

        if (newtonJoint_ != nullptr) {
            physicsWorld_->addToFreeQueue(newtonJoint_);
            newtonJoint_ = nullptr;
        }
    }



    void NewtonConstraint::AddJointReferenceToBody(RigidBody* rigBody)
    {

        if (!rigBody->connectedConstraints_.Contains(this))
            rigBody->connectedConstraints_.Insert(this);

    }


    void NewtonConstraint::RemoveJointReferenceFromBody(RigidBody* rigBody)
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

            RigidBody* rigBody = node->GetComponent<RigidBody>();
            if (rigBody) {
                ownBody_ = rigBody;
                ownBodyId_ = ownBody_->GetID();
            }
           
            if(physicsWorld_)
                physicsWorld_->addConstraint(this);

            AddJointReferenceToBody(ownBody_);

            node->AddListener(this);

        }
        else
        {
            if(!ownBody_.Expired())
                RemoveJointReferenceFromBody(ownBody_);

            ownBody_ = nullptr;
            if (physicsWorld_)
                physicsWorld_->removeConstraint(this);

            freeInternal();

        }
    }

    void NewtonConstraint::OnNodeSetEnabled(Node* node)
    {
        MarkDirty();
    }

}
