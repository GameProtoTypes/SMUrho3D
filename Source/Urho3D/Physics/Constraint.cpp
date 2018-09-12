#include "Constraint.h"
#include "RigidBody.h"
#include "PhysicsWorld.h"
#include "Core/Context.h"
#include "Scene/Component.h"
#include "Graphics/DebugRenderer.h"

#include "Scene/Scene.h"
#include "dCustomFixDistance.h"
#include "Newton.h"
#include "NewtonDebugDrawing.h"
#include "IO/Log.h"
namespace Urho3D {
    Constraint::Constraint(Context* context) : Component(context)
    {

    }

    Constraint::~Constraint()
    {
    }

    void Constraint::RegisterObject(Context* context)
    {
        context->RegisterFactory<Constraint>(DEF_PHYSICS_CATEGORY.CString());
        URHO3D_COPY_BASE_ATTRIBUTES(Component);

        URHO3D_ACCESSOR_ATTRIBUTE("Other Body ID", GetOtherBodyId, SetOtherBodyById, unsigned, 0, AM_DEFAULT | AM_COMPONENTID);
        URHO3D_ACCESSOR_ATTRIBUTE("Other Body Frame Position", GetOtherPosition, SetOtherPosition, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Other Body Frame Rotation", GetOtherRotation, SetOtherRotation, Quaternion, Quaternion::IDENTITY, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Body Frame Position", GetPosition, SetPosition, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Body Frame Rotation", GetRotation, SetRotation, Quaternion, Quaternion::IDENTITY, AM_DEFAULT);

    }

    void Constraint::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        //draw 2 part line from one frame to the other. Black touching own body and graw touching other body.
        if (ownBody_) {
           
            if (otherBody_) {
                Vector3 midPoint = (otherBody_->GetNode()->LocalToWorld(otherPosition_) + ownBody_->GetNode()->LocalToWorld(position_))*0.5f;
                debug->AddLine(ownBody_->GetNode()->LocalToWorld(position_), midPoint, Color::BLACK, depthTest);
                debug->AddLine(midPoint, otherBody_->GetNode()->LocalToWorld(otherPosition_), Color::GRAY, depthTest);

                //also add line from node center to node center - to make it clear that bodies are connected via joint in cases wheras the 2 world frames do not differ in world cordinates.
                debug->AddLine(ownBody_->GetNode()->GetWorldPosition(), otherBody_->GetNode()->GetWorldPosition(), Color::MAGENTA, depthTest);
            }
            else
            {   //draw from own body frame to world.
                Vector3 midPoint = (otherPosition_ + ownBody_->GetNode()->LocalToWorld(position_))*0.5f;
                debug->AddLine(ownBody_->GetNode()->LocalToWorld(position_), midPoint, Color::BLACK, depthTest);
                debug->AddLine(midPoint, otherPosition_, Color::GRAY, depthTest);
            }
        }


        //draw the frames.
        const float axisLengths = 0.5f;
        Vector3 xAxis, yAxis, zAxis;
        xAxis = Vector3(axisLengths, 0, 0);
        yAxis = Vector3(0, axisLengths, 0);
        zAxis = Vector3(0, 0, axisLengths);

        Vector3 xAxisOwn = (GetNode()->GetWorldRotation() * rotation_) * xAxis;
        Vector3 yAxisOwn = (GetNode()->GetWorldRotation() * rotation_) * yAxis;
        Vector3 zAxisOwn = (GetNode()->GetWorldRotation() * rotation_) * zAxis;

        Vector3 xAxisOther = xAxis;
        Vector3 yAxisOther = yAxis;
        Vector3 zAxisOther = zAxis;

        if (otherBody_) {
            xAxisOther = (otherBody_->GetNode()->GetWorldRotation() * otherRotation_) * xAxis;
            yAxisOther = (otherBody_->GetNode()->GetWorldRotation() * otherRotation_) * yAxis;
            zAxisOther = (otherBody_->GetNode()->GetWorldRotation() * otherRotation_) * zAxis;
        }
        else
        {
            xAxisOther = (otherRotation_) * xAxis;
            yAxisOther = (otherRotation_) * yAxis;
            zAxisOther = (otherRotation_) * zAxis;
        }

        Vector3 ownPosWorld = ownBody_->GetNode()->LocalToWorld(position_);
        Vector3 otherPosWorld;
        if (otherBody_)
            otherPosWorld = otherBody_->GetNode()->LocalToWorld(otherPosition_);
        else
            otherPosWorld = otherPosition_;

        debug->AddLine(ownPosWorld, ownPosWorld + xAxisOwn, Color::RED, depthTest);
        debug->AddLine(ownPosWorld, ownPosWorld + yAxisOwn, Color::GREEN, depthTest);
        debug->AddLine(ownPosWorld, ownPosWorld + zAxisOwn, Color::BLUE, depthTest);

        debug->AddLine(otherPosWorld, otherPosWorld + xAxisOther, Color::RED, depthTest);
        debug->AddLine(otherPosWorld, otherPosWorld + yAxisOther, Color::GREEN, depthTest);
        debug->AddLine(otherPosWorld, otherPosWorld + zAxisOther, Color::BLUE, depthTest);


        //draw the special joint stuff given to us by newton
        UrhoNewtonDebugDisplay debugDisplay(debug, depthTest);
        if (newtonJoint_)
        {
            newtonJoint_->Debug(&debugDisplay);
        }

    }

    void Constraint::SetDisableCollision(bool disable)
    {
        enableBodyCollision_ = !disable;
        MarkDirty();
    }

    void Constraint::SetOtherBody(RigidBody* body)
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


    void Constraint::SetOtherBodyById(unsigned bodyId)
    {
        otherBodyId_ = bodyId;
        //resolve to body later.
        MarkDirty();
    }

    void Constraint::SetPosition(const Vector3& position)
    {
        position_ = position;
        MarkDirty();
    }


    void Constraint::SetRotation(const Quaternion& rotation)
    {
        rotation_ = rotation;
        MarkDirty();
    }


    void Constraint::SetWorldPosition(const Vector3& worldPosition)
    {
        position_ = node_->WorldToLocal(worldPosition);
        MarkDirty();
    }

    void Constraint::SetWorldRotation(const Quaternion& worldRotation)
    {
        rotation_ = node_->WorldToLocal(worldRotation);
        MarkDirty();
    }

    void Constraint::SetOtherPosition(const Vector3& position)
    {
        otherPosition_ = position;
        MarkDirty();
    }


    void Constraint::SetOtherRotation(const Quaternion& rotation)
    {
        otherRotation_ = rotation;
        MarkDirty();
    }


    void Constraint::SetOtherWorldPosition(const Vector3& position)
    {
        if (otherBody_)
        {
            otherPosition_ = otherBody_->GetNode()->WorldToLocal(position);
        }
        else
            otherPosition_ = position;

        MarkDirty();
    }

    void Constraint::SetOtherWorldRotation(const Quaternion& rotation)
    {
        if (otherBody_)
        {
            otherRotation_ = otherBody_->GetNode()->WorldToLocal(rotation);
        }
        else
            otherRotation_ = rotation;

        MarkDirty();
    }

    NewtonBody* Constraint::GetOwnNewtonBody() const
    {
        return ownBody_->GetNewtonBody();
    }

    NewtonBody* Constraint::GetOtherNewtonBody() const
    {
        if (otherBody_)
            return otherBody_->GetNewtonBody();
        else
            return nullptr;
    }

    Urho3D::Matrix3x4 Constraint::GetOwnWorldFrame() const
    {
        Vector3 nodeScale = node_->GetScale();

        Matrix3x4 frame = node_->LocalToWorld(Matrix3x4(position_, rotation_, 1.0f));

        return frame;
    }

    Urho3D::Matrix3x4 Constraint::GetOtherWorldFrame() const
    {
        if (otherBody_) {

            Node* otherNode = otherBody_->GetNode();
            Matrix3x4 frame = otherNode->LocalToWorld(Matrix3x4(otherPosition_, otherRotation_, 1.0f));


            return frame;
        }
        else
            return Matrix3x4(otherPosition_, otherRotation_, 1.0f);
    }

    void Constraint::OnSetEnabled()
    {
        MarkDirty();
    }

    void Constraint::reEvalConstraint()
    {
        //resolve other body id to component
        otherBody_ = static_cast<RigidBody*>(GetScene()->GetComponent(otherBodyId_));

        if(otherBody_)
            LogNodeScaleWarning(otherBody_->GetNode());

        LogNodeScaleWarning(node_);

        if (!IsEnabledEffective()) {
            freeInternal();
        }
        else if (ownBody_ && ownBody_->GetNode() && ownBody_->GetNewtonBody()) {
            freeInternal();
            buildConstraint();


            NewtonJointSetCollisionState((NewtonJoint*)newtonJoint_, enableBodyCollision_);
            //NewtonJointSetStiffness((NewtonJoint*)newtonJoint_, stiffness_);
            newtonJoint_->SetStiffness(stiffness_);
            newtonJoint_->SetSolverModel(solverIterations_);

        }
        else//we dont have own body so free the joint..
        {
            freeInternal();
        }






        MarkDirty(false);
    }

    void Constraint::buildConstraint()
    {
        /// ovverride in derived classes.
    }


    void Constraint::freeInternal()
    {

        if (newtonJoint_ != nullptr) {
            physicsWorld_->addToFreeQueue(newtonJoint_);
            newtonJoint_ = nullptr;
        }
    }



    void Constraint::AddJointReferenceToBody(RigidBody* rigBody)
    {

        if (!rigBody->connectedConstraints_.Contains(this))
            rigBody->connectedConstraints_.Insert(this);

    }


    void Constraint::RemoveJointReferenceFromBody(RigidBody* rigBody)
    {

        if (rigBody->connectedConstraints_.Contains(this))
            rigBody->connectedConstraints_.Erase(this);

    }

    void Constraint::OnNodeSet(Node* node)
    {
        if (node)
        {
            //auto create physics world similar to rigid body.
            physicsWorld_ = node->GetScene()->GetOrCreateComponent<PhysicsWorld>();

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

    void Constraint::OnNodeSetEnabled(Node* node)
    {
        MarkDirty();
    }

    void Constraint::LogNodeScaleWarning(Node* node)
    {

        if ((node->GetWorldScale() - Vector3::ONE).Length() > M_LARGE_EPSILON*100.0f) {

            URHO3D_LOGWARNING("Contraints Do Not Support contraining Rigid Bodies with node world scales other than 1");
        }

    }

}
