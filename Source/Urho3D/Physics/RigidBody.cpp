#include "../Physics/UrhoNewtonPhysicsWorld.h"
#include "../Physics/NewtonCollisionShape.h"
#include "../Physics/RigidBody.h"
#include "../Core/Context.h"
#include "../Graphics/Model.h"
#include "IO/Log.h"
#include "Scene/Scene.h"
#include "Scene/Node.h"
#include "dMatrix.h"
#include "Newton.h"
#include "NewtonDebugDrawing.h"
#include "UrhoNewtonConversions.h"
#include "Scene/SceneEvents.h"
#include "Engine/Engine.h"
#include "Core/Profiler.h"
#include "Graphics/VisualDebugger.h"
#include "Core/Object.h"
#include "dQuaternion.h"
#include "NewtonConstraint.h"


namespace Urho3D {






    RigidBody::RigidBody(Context* context) : Component(context)
    {
        SubscribeToEvent(E_NODEADDED, URHO3D_HANDLER(RigidBody, HandleNodeAdded));
        SubscribeToEvent(E_NODEREMOVED, URHO3D_HANDLER(RigidBody, HandleNodeRemoved));
    }

    RigidBody::~RigidBody()
    {

    }

    void RigidBody::RegisterObject(Context* context)
    {
        context->RegisterFactory<RigidBody>(DEF_PHYSICS_CATEGORY.CString());

        URHO3D_COPY_BASE_ATTRIBUTES(Component);


        URHO3D_ACCESSOR_ATTRIBUTE("MassScale", GetMassScale, SetMassScale, float, 1.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Linear Velocity", GetLinearVelocity, SetLinearVelocity, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Angular Velocity", GetAngularVelocity, SetAngularVelocity, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Inherit Collision Node Scales", GetInheritNodeScale, SetInheritNodeScale, bool, true, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Continuous Collision", GetContinuousCollision, SetContinuousCollision, bool, false, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Linear Damping", GetLinearDamping, SetLinearDamping, float, 0.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Angular Damping", GetAngularDamping, SetAngularDamping, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Net Force", Vector3, netForce_, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Net Torque", Vector3, netTorque_, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ATTRIBUTE("Is Scene Root Body", bool, sceneRootBodyMode_, false, AM_DEFAULT);
    }


    void RigidBody::SetMassScale(float massDensityScale)
    {
        if (massScale_ != massDensityScale) {
            massScale_ = massDensityScale;
            MarkDirty(true);
        }
    }


    void RigidBody::SetLinearVelocity(const Vector3& velocity)
    {
        nextLinearVelocity_ = velocity;
        nextLinearVelocityNeeded_ = true;
    }


    void RigidBody::SetAngularVelocity(const Vector3& angularVelocity)
    {
        nextAngularVelocity_ = angularVelocity;
        nextAngularVelocityNeeded_ = true;
    }

    void RigidBody::SetLinearDamping(float dampingFactor)
    {
        if (linearDampening_ != dampingFactor) {
            linearDampening_ = dampingFactor;
        }
    }

    void RigidBody::SetAngularDamping(const Vector3& angularDamping)
    {
        if (angularDamping != angularDamping) {
            angularDampening_ = angularDamping;
        }
    }

    void RigidBody::SetInternalLinearDamping(float damping)
    {
        if (linearDampeningInternal_ != damping) {
            linearDampeningInternal_ = damping;

            if (newtonBody_)
            {
                NewtonBodySetLinearDamping(newtonBody_, linearDampeningInternal_);
            }
        }
    }

    void RigidBody::SetInternalAngularDamping(const Vector3& angularDamping)
    {
        angularDampeningInternal_ = angularDamping;
        if (newtonBody_)
        {
            NewtonBodySetAngularDamping(newtonBody_, &UrhoToNewton(angularDampeningInternal_)[0]);
        }
    }

    void RigidBody::SetInheritNodeScale(bool enable /*= true*/)
    {
        if (inheritCollisionNodeScales_ != enable) {
            inheritCollisionNodeScales_ = enable;
            MarkDirty();//we need to rebuild for this chang.e
        }

    }

    void RigidBody::SetContinuousCollision(bool sweptCollision)
    {
        if (continuousCollision_ != sweptCollision) {
            continuousCollision_ = sweptCollision;
            if (newtonBody_) {
                NewtonBodySetContinuousCollisionMode(newtonBody_, sweptCollision);
            }
        }
    }


    void RigidBody::SetAutoSleep(bool enableAutoSleep)
    {
        if (autoSleep_ != enableAutoSleep)
        {
            autoSleep_ = enableAutoSleep;
            if (newtonBody_)
            {
                NewtonBodySetAutoSleep(newtonBody_, autoSleep_);
                NewtonBodySetSleepState(newtonBody_, !autoSleep_);
            }
        }
    }

    void RigidBody::SetIsSceneRootBody(bool enable)
    {
        if (sceneRootBodyMode_ != enable) {
            sceneRootBodyMode_ = enable;
            MarkDirty(true);
        }
    }

    void RigidBody::DrawDebugGeometry(DebugRenderer* debug, bool depthTest, bool showAABB /*= true*/, bool showCollisionMesh /*= true*/, bool showCenterOfMass /*= true*/, bool showContactForces /*= true*/)
    {
        Component::DrawDebugGeometry(debug, depthTest);
        if (newtonBody_) {
            if (showAABB) NewtonDebug_BodyDrawAABB(newtonBody_, debug, depthTest);
            if (showCollisionMesh) NewtonDebug_BodyDrawCollision(newtonBody_, debug, depthTest);
            if (showCenterOfMass) NewtonDebug_BodyDrawCenterOfMass(newtonBody_, debug, depthTest);
            if (showContactForces)  NewtonDebug_BodyDrawContactForces(newtonBody_, 0., debug, depthTest);
        }
    }


    void RigidBody::MarkDirty(bool dirty)
    {
        needsRebuilt_ = dirty;
    }

    void RigidBody::MarkInternalTransformDirty(bool dirty)
    {
        transformDirty_ = dirty;
    }

    bool RigidBody::GetInternalTransformDirty()
    {
        return transformDirty_;
    }


    void RigidBody::OnSetEnabled()
    {
        MarkDirty(true);
    }

    void RigidBody::freeBody()
    {
        if (newtonBody_ != nullptr) {
            physicsWorld_->addToFreeQueue(newtonBody_);
            newtonBody_ = nullptr;
        }


        //also free the compound collision if there is one
        if (compoundCollision_)
        {
            physicsWorld_->addToFreeQueue(compoundCollision_);
            compoundCollision_ = nullptr;
        }
    }



    void RigidBody::reBuildBody()
    {
        URHO3D_PROFILE_FUNCTION();
        freeBody();

        if (!IsEnabledEffective())
            return;


        //evaluate child nodes (+this node) and see if there are more collision shapes - if so create a compound collision.
        PODVector<NewtonCollisionShape*> childCollisionShapes;
        node_->GetDerivedComponents<NewtonCollisionShape>(childCollisionShapes, true, true);//includes this node.
        
        PODVector<NewtonCollisionShape*> filteredList;
        //if scene root body - filter out shapes with no root rigid body (except the scene root of course)
        if (sceneRootBodyMode_)
        {
            for (NewtonCollisionShape* col : childCollisionShapes)
            {
                PODVector<RigidBody*> parentRigBodies;
                GetRootRigidBodies(parentRigBodies, col->GetNode(), false);
                if (parentRigBodies.Size() == 0)
                    filteredList += col;
            }
            childCollisionShapes = filteredList;
        }

        //filter out shapes that are not enabled.
        filteredList.Clear();
        for (NewtonCollisionShape* col : childCollisionShapes)
        {
            if (col->IsEnabledEffective())
                filteredList += col;
        }
        childCollisionShapes = filteredList;


        if (childCollisionShapes.Size())
        {  
            NewtonCollision* resolvedCollision = nullptr;

            if (compoundCollision_) {
                NewtonDestroyCollision(compoundCollision_);
                compoundCollision_ = nullptr;
            }
            bool compoundNeeded = (childCollisionShapes.Size() > 1);

            if (compoundNeeded) {
                if (sceneRootBodyMode_)
                    compoundCollision_ = NewtonCreateSceneCollision(physicsWorld_->GetNewtonWorld(), 0);//internally the same as a regular compond with some flags enabled..
                else
                    compoundCollision_ = NewtonCreateCompoundCollision(physicsWorld_->GetNewtonWorld(), 0);



                NewtonCompoundCollisionBeginAddRemove(compoundCollision_);
            }
            float accumMass = 0.0f;

            for (NewtonCollisionShape* colComp : childCollisionShapes)
            {
                NewtonCollision* curNewtCollision = colComp->GetNewtonCollision();
                NewtonCollision* usedCollision = nullptr;


                //check if there is a rigid body on the same node as colComp - if so (and it isnt this RigidBody) free the internal body of it.
                if (colComp->GetNode() != node_ && colComp->GetNode()->HasComponent<RigidBody>())
                    colComp->GetComponent<RigidBody>()->freeBody();


                if (compoundNeeded)
                    usedCollision = NewtonCollisionCreateInstance(curNewtCollision);
                else
                    usedCollision = curNewtCollision;



                Matrix3x4 colWorldTransformNoScale = Matrix3x4(colComp->GetNode()->GetWorldPosition(), colComp->GetNode()->GetWorldRotation(), 1.0f);
                Matrix3x4 thisWorldTransformNoScale = Matrix3x4(node_->GetWorldPosition(), node_->GetWorldRotation(), 1.0f);
                Matrix3x4 thisLocalTransformNoScale = Matrix3x4(node_->GetPosition(), node_->GetRotation(), 1.0f);


                Matrix3x4 colLocalOffsetTransform = colComp->getInternalOffsetMatrix()*colComp->GetOffsetMatrix();

                Matrix3x4 colWorldOffset = colWorldTransformNoScale * colLocalOffsetTransform;
                Matrix3x4 colLocalToThisNode = thisWorldTransformNoScale.Inverse()*colWorldOffset;



                dMatrix localTransform = UrhoToNewton(colLocalToThisNode);
                NewtonCollisionSetMatrix(usedCollision, &localTransform[0][0]);//set the collision matrix with translation and rotation data only.

                if (inheritCollisionNodeScales_)
                {
                    
                    Vector3 scale = colComp->GetNode()->GetWorldScale();


                    scale = (colComp->internalRotOffset_*colComp->GetRotationOffset()).Inverse()*scale;


                    NewtonCollisionSetScale(usedCollision, scale.x_, scale.y_, scale.z_);//then scale.
                }
                float vol = NewtonConvexCollisionCalculateVolume(usedCollision);
                accumMass += vol*1.0f;//#todo 1.0f should be mass density of material attached to collision shape.

                if (compoundNeeded) {

                    if(sceneRootBodyMode_)
                        NewtonSceneCollisionAddSubCollision(compoundCollision_, usedCollision);
                    else
                        NewtonCompoundCollisionAddSubCollision(compoundCollision_, usedCollision);

                    NewtonDestroyCollision(usedCollision);
                }
                else
                    resolvedCollision = usedCollision;

            }

            if (compoundNeeded) {

                NewtonCompoundCollisionEndAddRemove(compoundCollision_);

                resolvedCollision = compoundCollision_;
            }

            

            //create the body at node transform
            Matrix4 transform;
            transform.SetTranslation(node_->GetWorldPosition());
            transform.SetRotation(node_->GetWorldRotation().RotationMatrix());
            dMatrix mat = UrhoToNewton(transform);

            newtonBody_ = NewtonCreateDynamicBody(physicsWorld_->GetNewtonWorld(), resolvedCollision, &mat[0][0]);
            NewtonBodySetCollision(newtonBody_, resolvedCollision);
            dVector inertia;
            dVector com;
            if (compoundNeeded) {
                NewtonConvexCollisionCalculateInertialMatrix(resolvedCollision, &inertia[0], &com[0]);
            }


            mass_ = accumMass * massScale_;
            if (sceneRootBodyMode_)
                mass_ = 0;

            if (compoundNeeded) {
                NewtonBodySetMassMatrix(newtonBody_, mass_, inertia[0], inertia[1], inertia[2]);
                NewtonBodySetCentreOfMass(newtonBody_, &com[0]);
            }
            else
                NewtonBodySetMassProperties(newtonBody_, mass_, resolvedCollision);



            NewtonBodySetUserData(newtonBody_, (void*)this);

            NewtonBodySetContinuousCollisionMode(newtonBody_, continuousCollision_);

            //ensure newton damping is 0 because we apply our own as a force.
            NewtonBodySetLinearDamping(newtonBody_, linearDampeningInternal_);
            NewtonBodySetAngularDamping(newtonBody_, &UrhoToNewton(angularDampeningInternal_)[0]);

            //set sleep state.
            NewtonBodySetAutoSleep(newtonBody_, autoSleep_);
            NewtonBodySetSleepState(newtonBody_, !autoSleep_);


            //assign callbacks
            NewtonBodySetForceAndTorqueCallback(newtonBody_, Newton_ApplyForceAndTorqueCallback);
            NewtonBodySetTransformCallback(newtonBody_, Newton_SetTransformCallback); 
            NewtonBodySetDestructorCallback(newtonBody_, Newton_DestroyBodyCallback);
        }
    }





    void RigidBody::bakeForceAndTorque()
    {

    }

    void RigidBody::OnNodeSet(Node* node)
    {
        if (node)
        {

            //Auto-create a physics world on the scene if it does not yet exist.
            physicsWorld_ = WeakPtr<UrhoNewtonPhysicsWorld>(GetScene()->GetOrCreateComponent<UrhoNewtonPhysicsWorld>());

            physicsWorld_->addRigidBody(this);

            node->AddListener(this);

            SubscribeToEvent(node, E_NODETRANSFORMCHANGE, URHO3D_HANDLER(RigidBody, HandleNodeTransformChange));
        }
        else
        {

            if (physicsWorld_)
                physicsWorld_->removeRigidBody(this);

            //remove any connected constraints.
            for (NewtonConstraint* constraint : connectedConstraints_) {
                constraint->Remove();
            }

            freeBody();
            UnsubscribeFromEvent(E_NODETRANSFORMCHANGE);
        }

    }

    void RigidBody::OnSceneSet(Scene* scene)
    {

    }
    void RigidBody::HandleNodeAdded(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeAdded::P_NODE].GetPtr());
        Node* newParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());

        if (node == node_)
        {
            OnPhysicsNodeAdded(node);
        }
    }

    void RigidBody::HandleNodeRemoved(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeRemoved::P_NODE].GetPtr());
        if (node == node_)
        {
            Node* oldParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());



            if (oldParent)
            {
                OnPhysicsNodeRemoved(oldParent);
            }
            else
            {
                URHO3D_LOGINFO("should not happen");
            }
        }
    }


   

    void RigidBody::HandleNodeTransformChange(StringHash event, VariantMap& eventData)
    {
        RigidBody* parentRigidBody = node_->GetParentComponent<RigidBody>(true);
        if (newtonBody_ && !parentRigidBody) {
            //the node's transform has explictly been changed.  set the rigid body transform to the same transform.
            //Ignore scale.
            Matrix3x4 mat(eventData[NodeTransformChange::P_NEW_POSITION].GetVector3(),
                eventData[NodeTransformChange::P_NEW_ORIENTATION].GetQuaternion(),
                Vector3::ONE);

            NewtonBodySetMatrix(newtonBody_, &UrhoToNewton(mat.ToMatrix4())[0][0]);
        }
        else
        {
            //handle case where node is child of other rigid body (part of compound).
            PODVector<RigidBody*> parentRigBodies;
            GetRootRigidBodies(parentRigBodies, node_, false);
            parentRigBodies.Back()->MarkDirty();

        }
    }



    void RigidBody::applyDefferedActions()
    {
        if (nextLinearVelocityNeeded_)
        {
            if (newtonBody_)
            {
                NewtonBodySetVelocity(newtonBody_, &UrhoToNewton(nextLinearVelocity_)[0]);
            }
            nextLinearVelocityNeeded_ = false;
        }
        if (nextAngularVelocityNeeded_)
        {
            if (newtonBody_)
            {
                NewtonBodySetOmega(newtonBody_, &UrhoToNewton(nextAngularVelocity_)[0]);
            }
            nextAngularVelocityNeeded_ = false;
        }



    }

    void RigidBody::OnNodeSetEnabled(Node* node)
    {
        if (node == node_)
        {
            MarkDirty(true);
        }
    }

    void RigidBody::AddWorldForce(const Vector3& force)
    {
        AddWorldForce(force, Vector3::ZERO);
    }

    void RigidBody::AddWorldForce(const Vector3& force, const Vector3& localPosition)
    {
        netForce_ += force;
        netTorque_ += localPosition.CrossProduct(node_->WorldToLocal(force));
        bakeForceAndTorque();
    }

    void RigidBody::AddWorldTorque(const Vector3& torque)
    {
        netTorque_ += torque;
        bakeForceAndTorque();
    }

    void RigidBody::AddLocalForce(const Vector3& force)
    {
        AddWorldForce(node_->LocalToWorld(force));
    }

    void RigidBody::AddLocalForce(const Vector3& force, const Vector3& localPosition)
    {
        AddWorldForce(node_->LocalToWorld(force), localPosition);
    }

    void RigidBody::AddLocalTorque(const Vector3& torque)
    {
        AddWorldTorque(node_->LocalToWorld(torque));
    }

    void RigidBody::ResetForces()
    {
        netForce_ = Vector3(0, 0, 0);
        netTorque_ = Vector3(0, 0, 0);
        bakeForceAndTorque();
    }

    void RigidBody::AddImpulse(const Vector3& localPosition, const Vector3& targetVelocity)
    {
        if(newtonBody_)
            NewtonBodyAddImpulse(newtonBody_, &UrhoToNewton(targetVelocity)[0], &UrhoToNewton(node_->LocalToWorld(localPosition))[0], GSS<Engine>()->GetUpdateTimeGoalMs()*0.001f);

        
    }

    Vector3 RigidBody::GetNetForce()
    {
        return netForce_;
    }


    Urho3D::Vector3 RigidBody::GetNetTorque()
    {
        return netTorque_;
    }

    NewtonCollision* RigidBody::GetEffectiveNewtonCollision() const
    {
        if (compoundCollision_)
            return compoundCollision_;
        else
        {
            if (node_->GetComponent<NewtonCollisionShape>()) {
                return node_->GetComponent<NewtonCollisionShape>()->GetNewtonCollision();
            }
            return nullptr;
        }
    }

    Urho3D::Vector3 RigidBody::GetLinearVelocity(TransformSpace space /*= TS_WORLD*/) const
{
        if (newtonBody_) {
            dVector vel;
            NewtonBodyGetVelocity(newtonBody_, &vel[0]);

            if (space == TS_WORLD)
            {
                return NewtonToUrhoVec3(vel);
            }
            else if (space == TS_LOCAL)
            {
                return node_->WorldToLocal(NewtonToUrhoVec3(vel));
            }
            else if (space == TS_PARENT)
            {
                return node_->GetParent()->WorldToLocal(NewtonToUrhoVec3(vel));
            }

            return NewtonToUrhoVec3(vel);
        }
        else
            return Vector3::ZERO;
    }

    Urho3D::Vector3 RigidBody::GetAngularVelocity(TransformSpace space /*= TS_WORLD*/) const
    {
        if (newtonBody_) {
            dVector angularVel;
            NewtonBodyGetOmega(newtonBody_, &angularVel[0]);

            if (space == TS_WORLD)
            {
                return NewtonToUrhoVec3(angularVel);
            }
            else if(space == TS_LOCAL)
            {
                return node_->WorldToLocal(NewtonToUrhoVec3(angularVel));
            }
            else if (space == TS_PARENT)
            {
                return node_->GetParent()->WorldToLocal(NewtonToUrhoVec3(angularVel));
            }
            return NewtonToUrhoVec3(angularVel);
        }
        else
            return Vector3::ZERO;
    }


    Vector3 RigidBody::GetAcceleration()
    {
        if (newtonBody_) {
            dVector vel;
            NewtonBodyGetAcceleration(newtonBody_, &vel[0]);
            return NewtonToUrhoVec3(vel);
        }
        else
            return Vector3::ZERO;
    }

    Vector3 RigidBody::GetCenterOfMassPosition()
    {
        dVector pos;
        NewtonBodyGetPosition(newtonBody_, &pos[0]);
        return NewtonToUrhoVec3(pos);
    }

    Quaternion RigidBody::GetCenterOfMassRotation()
    {
        dQuaternion quat;
        NewtonBodyGetRotation(newtonBody_, &quat.m_q0);
        return NewtonToUrhoQuat(quat);
    }

    void RigidBody::GetConnectedContraints(PODVector<NewtonConstraint*>& contraints)
    {
        contraints.Clear();
        for (auto i = connectedConstraints_.Begin(); i != connectedConstraints_.End(); ++i)
        {
            contraints += *i;
        }
    }

    void RigidBody::ApplyTransform()
    {
        if (!newtonBody_)
            return;

        dVector pos;
        dQuaternion quat;
        NewtonBodyGetPosition(newtonBody_, &pos[0]);
        NewtonBodyGetRotation(newtonBody_, &quat.m_q0);

        bool enableTEvents = node_->GetEnableTransformEvents();
        if(enableTEvents)
            node_->SetEnableTransformEvents(false);


        node_->SetWorldTransform(NewtonToUrhoVec3(pos), NewtonToUrhoQuat(quat));

        node_->SetEnableTransformEvents(enableTEvents);
    }


    void RigidBody::GetForceAndTorque(Vector3& force, Vector3& torque)
    {
        URHO3D_PROFILE("GetForceAndTorque");


        //basic damping forces (clamp to some reasonable values)
        float linearDampingClamped = Urho3D::Clamp<float>(linearDampening_, 0.0f, 0.3f);
        Vector3 angularDampingClamped = Urho3D::VectorClamp(angularDampening_, Vector3::ZERO, Vector3::ONE * 0.3f);


        //basic velocity damping forces
        Vector3 velocity = GetLinearVelocity();
        Vector3 linearDampingForce = -velocity.Normalized()*(velocity.LengthSquared())*linearDampingClamped * mass_;

        //basic angular damping forces
        Vector3 angularVelocity = GetAngularVelocity();
        Vector3 angularDampingTorque = Vector3::ZERO;// -angularVelocity.Normalized()*(angularVelocity.LengthSquared())*angularDampingClamped * mass_;

        force = linearDampingForce + netForce_;
        torque = angularDampingTorque + netTorque_;

    }

}
