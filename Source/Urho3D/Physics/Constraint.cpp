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


    const char* solveModeNames[] =
    {
        "SOLVE_MODE_DEFAULT",
        "SOLVE_MODE_ITERATIVE",
        "SOLVE_MODE_KINEMATIC_LOOP",
        nullptr
    };


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

        URHO3D_ENUM_ACCESSOR_ATTRIBUTE("Solver Iterations", GetSolveMode, SetSolveMode, CONSTRAINT_SOLVE_MODE, solveModeNames, SOLVE_MODE_DEFAULT, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Stiffness", GetStiffness, SetStiffness, float, 0.7f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Other Body ID", GetOtherBodyId, SetOtherBodyById, unsigned, 0, AM_DEFAULT | AM_COMPONENTID);
        URHO3D_ACCESSOR_ATTRIBUTE("Other Body Frame Position", GetOtherPosition, SetOtherPosition, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Other Body Frame Rotation", GetOtherRotation, SetOtherRotation, Quaternion, Quaternion::IDENTITY, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Body Frame Position", GetOwnPosition, SetOwnPosition, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Body Frame Rotation", GetOwnRotation, SetOwnRotation, Quaternion, Quaternion::IDENTITY, AM_DEFAULT);

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

        Color darkRed = Color::RED.Lerp(Color::BLACK, 0.5f);
        Color darkGreen = Color::GREEN.Lerp(Color::BLACK, 0.5f);
        Color darkBlue = Color::BLUE.Lerp(Color::BLACK, 0.5f);

        debug->AddLine(ownPosWorld, ownPosWorld + xAxisOwn, Color::RED, depthTest);
        debug->AddLine(ownPosWorld, ownPosWorld + yAxisOwn, Color::GREEN, depthTest);
        debug->AddLine(ownPosWorld, ownPosWorld + zAxisOwn, Color::BLUE, depthTest);

        debug->AddLine(otherPosWorld, otherPosWorld + xAxisOther, darkRed, depthTest);
        debug->AddLine(otherPosWorld, otherPosWorld + yAxisOther, darkGreen, depthTest);
        debug->AddLine(otherPosWorld, otherPosWorld + zAxisOther, darkBlue, depthTest);


        //draw the special joint stuff given to us by newton
        UrhoNewtonDebugDisplay debugDisplay(debug, depthTest);
        if (newtonJoint_)
        {
            newtonJoint_->Debug(&debugDisplay);//#todo this covers up the 2 frames above - maybe alter inside newton instead?
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

    void Constraint::SetWorldPosition(const Vector3& position)
    {
        SetOwnWorldPosition(position);
        SetOtherWorldPosition(position);
    }

    void Constraint::SetWorldRotation(const Quaternion& rotation)
    {
        SetOwnWorldRotation(rotation);
        SetOtherWorldRotation(rotation);
    }

    void Constraint::SetOwnPosition(const Vector3& position)
    {
        position_ = position;
        MarkDirty();
    }


    void Constraint::SetOwnRotation(const Quaternion& rotation)
    {
        rotation_ = rotation;
        MarkDirty();
    }


    void Constraint::SetOwnWorldPosition(const Vector3& worldPosition)
    {
       // Matrix3x4 worldTransform(ownBody_->GetNode()->GetWorldPosition(), ownBody_->GetNode()->GetWorldRotation(), 1.0f);
        position_ = ownBody_->GetNode()->GetWorldTransform().Inverse() *  worldPosition;
        MarkDirty();
    }

    void Constraint::SetOwnWorldRotation(const Quaternion& worldRotation)
    {
        Quaternion worldRot = ownBody_->GetNode()->GetWorldRotation();
        rotation_ = worldRot.Inverse() * worldRotation;
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
            otherPosition_ = otherBody_->GetNode()->GetWorldTransform().Inverse() * position;
        }
        else
            otherPosition_ = position;

        MarkDirty();
    }

    void Constraint::SetOtherWorldRotation(const Quaternion& rotation)
    {
        if (otherBody_)
        {
              Quaternion worldRot = otherBody_->GetNode()->GetWorldRotation();
              otherRotation_ = worldRot.Inverse() * rotation;
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

    Urho3D::Vector3 Constraint::GetOtherPosition() const
    {

       return otherPosition_;

    }

    Urho3D::Quaternion Constraint::GetOtherRotation() const
    {
        return otherRotation_;
    }

    Urho3D::Matrix3x4 Constraint::GetOwnWorldFrame(bool scaledPhysicsWorldFrame) const
    {

        //return a frame with no scale at the position and rotation in node space (or physics world space if scaledPhysicsWorldFrame is true)
        Matrix3x4 worldTransform = ownBody_->GetNode()->GetWorldTransform();
        Matrix3x4 worldTransformNoScale(ownBody_->GetNode()->GetWorldPosition(), ownBody_->GetNode()->GetWorldRotation(),1.0f);

        Vector3 pivotPoint;
        if (scaledPhysicsWorldFrame)
            pivotPoint = physicsWorld_->GetPhysicsScale() * (worldTransform * position_);
        else
            pivotPoint = (worldTransform * position_);

        Matrix3x4 frame = worldTransformNoScale * Matrix3x4(position_, rotation_, 1.0f);
        frame.SetTranslation(pivotPoint);


        return frame;
    }

    Urho3D::Matrix3x4 Constraint::GetOtherWorldFrame(bool scaledPhysicsWorldFrame) const
    {
        if (otherBody_) {

            //return a frame with no scale at the position and rotation in node space.
            Matrix3x4 worldTransform = otherBody_->GetNode()->GetWorldTransform();
            Matrix3x4 worldTransformNoScale(otherBody_->GetNode()->GetWorldPosition(), otherBody_->GetNode()->GetWorldRotation(), 1.0f);

            Vector3 pivotPoint;
            if (scaledPhysicsWorldFrame)
                pivotPoint = physicsWorld_->GetPhysicsScale() * (worldTransform * otherPosition_);
            else
                pivotPoint = (worldTransform * otherPosition_);

            Matrix3x4 frame = worldTransformNoScale * Matrix3x4(otherPosition_, otherRotation_, 1.0f);
            frame.SetTranslation(pivotPoint);

            return frame;
        }
        else
        {
            if(scaledPhysicsWorldFrame)
                return Matrix3x4(scaledPhysicsWorldFrame*otherPosition_, otherRotation_, 1.0f);
            else
                return Matrix3x4(otherPosition_, otherRotation_, 1.0f);

        }
    }

    void Constraint::OnSetEnabled()
    {
        MarkDirty();
    }

    void Constraint::reEvalConstraint()
    {
        //resolve other body id to component
        otherBody_ = static_cast<RigidBody*>(GetScene()->GetComponent(otherBodyId_));


        if (!IsEnabledEffective()) {
            freeInternal();
        }
        else if (ownBody_ && ownBody_->GetNode() && ownBody_->GetNewtonBody()) {
            freeInternal();

            bool goodToBuild = true;
            if (otherBody_)
            {
                if (otherBody_->GetEffectiveMass() <= 0.0f && ownBody_->GetEffectiveMass() <= 0.0f)
                    goodToBuild = false;

            }
            else
            {
                if (ownBody_->GetEffectiveMass() <= 0.0f)
                    goodToBuild = false;
            }

            if (goodToBuild) {
                buildConstraint();
            }
            else
            {
                URHO3D_LOGWARNING("Contraint must connect to at least 1 Rigid Body with mass greater than 0.");
            }

            if (newtonJoint_ != nullptr) {
                NewtonJointSetCollisionState((NewtonJoint*)newtonJoint_, enableBodyCollision_);
                //NewtonJointSetStiffness((NewtonJoint*)newtonJoint_, stiffness_);
                newtonJoint_->SetStiffness(stiffness_);
                newtonJoint_->SetSolverModel(solveMode_);
            }
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



}
