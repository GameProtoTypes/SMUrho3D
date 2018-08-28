#include "../Physics/UrhoNewtonPhysicsWorld.h"
#include "../Physics/NewtonCollisionShape.h"
#include "../Physics/NewtonRigidBody.h"
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


namespace Urho3D {






    NewtonRigidBody::NewtonRigidBody(Context* context) : Component(context)
    {
        SubscribeToEvent(E_NODEADDED, URHO3D_HANDLER(NewtonRigidBody, HandleNodeAdded));
        SubscribeToEvent(E_NODEREMOVED, URHO3D_HANDLER(NewtonRigidBody, HandleNodeRemoved));
    }

    NewtonRigidBody::~NewtonRigidBody()
    {

    }

    void NewtonRigidBody::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonRigidBody>(DEF_PHYSICS_CATEGORY.CString());

        URHO3D_ACCESSOR_ATTRIBUTE("MassScale", GetMassScale, SetMassScale, float, 1.0f, AM_DEFAULT);

       // URHO3D_ATTRIBUTE_EX("Mass", float, mass_, MarkBodyDirty, DEFAULT_MASS, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Friction", GetFriction, SetFriction, float, DEFAULT_FRICTION, AM_DEFAULT);
        //URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Anisotropic Friction", GetAnisotropicFriction, SetAnisotropicFriction, Vector3, Vector3::ONE,
         //   AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Rolling Friction", GetRollingFriction, SetRollingFriction, float, DEFAULT_ROLLING_FRICTION, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Restitution", GetRestitution, SetRestitution, float, DEFAULT_RESTITUTION, AM_DEFAULT);
        //URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Linear Velocity", GetLinearVelocity, SetLinearVelocity, Vector3, Vector3::ZERO,
        //    AM_DEFAULT | AM_LATESTDATA);
        //URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Angular Velocity", GetAngularVelocity, SetAngularVelocity, Vector3, Vector3::ZERO, AM_FILE);
        //URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Linear Factor", GetLinearFactor, SetLinearFactor, Vector3, Vector3::ONE, AM_DEFAULT);
        //URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Angular Factor", GetAngularFactor, SetAngularFactor, Vector3, Vector3::ONE, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Linear Damping", GetLinearDamping, SetLinearDamping, float, 0.0f, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Angular Damping", GetAngularDamping, SetAngularDamping, float, 0.0f, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Linear Rest Threshold", GetLinearRestThreshold, SetLinearRestThreshold, float, 0.8f, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Angular Rest Threshold", GetAngularRestThreshold, SetAngularRestThreshold, float, 1.0f, AM_DEFAULT);
        //URHO3D_ATTRIBUTE_EX("Collision Layer", int, collisionLayer_, MarkBodyDirty, DEFAULT_COLLISION_LAYER, AM_DEFAULT);
        //URHO3D_ATTRIBUTE_EX("Collision Mask", int, collisionMask_, MarkBodyDirty, DEFAULT_COLLISION_MASK, AM_DEFAULT);
        ///* URHO3D_ACCESSOR_ATTRIBUTE("Contact Threshold", GetContactProcessingThreshold, SetContactProcessingThreshold, float, BT_LARGE_FLOAT,
        //AM_DEFAULT);*/
        //URHO3D_ACCESSOR_ATTRIBUTE("CCD Radius", GetCcdRadius, SetCcdRadius, float, 0.0f, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("CCD Motion Threshold", GetCcdMotionThreshold, SetCcdMotionThreshold, float, 0.0f, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Network Angular Velocity", GetNetAngularVelocityAttr, SetNetAngularVelocityAttr, PODVector<unsigned char>,
        //    Variant::emptyBuffer, AM_NET | AM_LATESTDATA | AM_NOEDIT);
        //URHO3D_ENUM_ATTRIBUTE_EX("Collision Event Mode", collisionEventMode_, MarkBodyDirty, collisionEventModeNames, COLLISION_ACTIVE, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Use Gravity", GetUseGravity, SetUseGravity, bool, true, AM_DEFAULT);
        //URHO3D_ATTRIBUTE_EX("Is Kinematic", bool, kinematic_, MarkBodyDirty, false, AM_DEFAULT);
        //URHO3D_ATTRIBUTE_EX("Is Trigger", bool, trigger_, MarkBodyDirty, false, AM_DEFAULT);
        //URHO3D_ACCESSOR_ATTRIBUTE("Gravity Override", GetGravityOverride, SetGravityOverride, Vector3, Vector3::ZERO, AM_DEFAULT);


    }


    void NewtonRigidBody::SetMassScale(float massDensityScale)
    {
        if (massScale_ != massDensityScale) {
            massScale_ = massDensityScale;
            MarkDirty(true);
        }
    }


    void NewtonRigidBody::SetLinearVelocity(const Vector3& velocity)
    {
        nextLinearVelocity_ = velocity;
        nextLinearVelocityNeeded_ = true;
    }

    void NewtonRigidBody::SetLinearDamping(float dampingFactor)
    {
        if (linearDampening_ != dampingFactor) {
            linearDampening_ = dampingFactor;
        }
    }

    void NewtonRigidBody::SetAngularDamping(const Vector3& angularDamping)
    {
        if (angularDamping != angularDamping) {
            angularDampening_ = angularDamping;
        }
    }

    void NewtonRigidBody::SetInternalLinearDamping(float damping)
    {
        if (linearDampeningInternal_ != damping) {
            linearDampeningInternal_ = damping;

            if (newtonBody_)
            {
                NewtonBodySetLinearDamping(newtonBody_, linearDampeningInternal_);
            }
        }
    }

    void NewtonRigidBody::SetInternalAngularDamping(const Vector3& angularDamping)
    {
        angularDampeningInternal_ = angularDamping;
        if (newtonBody_)
        {
            NewtonBodySetAngularDamping(newtonBody_, &UrhoToNewton(angularDampeningInternal_)[0]);
        }
    }

    void NewtonRigidBody::SetInheritNodeScale(bool enable /*= true*/)
    {
        if (inheritCollisionNodeScales_ != enable) {
            inheritCollisionNodeScales_ = enable;
            MarkDirty();//we need to rebuild for this chang.e
        }

    }

    void NewtonRigidBody::SetContinuousCollision(bool sweptCollision)
    {
        if (continuousCollision_ != sweptCollision) {
            continuousCollision_ = sweptCollision;
            if (newtonBody_) {
                NewtonBodySetContinuousCollisionMode(newtonBody_, sweptCollision);
            }
        }
    }


    void NewtonRigidBody::SetIsSceneRootBody(bool enable)
    {
        if (sceneRootBodyMode_ != enable) {
            sceneRootBodyMode_ = enable;
            MarkDirty(true);
        }
    }

    void NewtonRigidBody::DrawDebugGeometry(DebugRenderer* debug, bool depthTest, bool showAABB /*= true*/, bool showCollisionMesh /*= true*/, bool showCenterOfMass /*= true*/, bool showContactForces /*= true*/)
    {
        Component::DrawDebugGeometry(debug, depthTest);
        if (newtonBody_) {
            if (showAABB) NewtonDebug_BodyDrawAABB(newtonBody_, debug, depthTest);
            if (showCollisionMesh) NewtonDebug_BodyDrawCollision(newtonBody_, debug, depthTest);
            if (showCenterOfMass) NewtonDebug_BodyDrawCenterOfMass(newtonBody_, debug, depthTest);
            if (showContactForces)  NewtonDebug_BodyDrawContactForces(newtonBody_, 0., debug, depthTest);
        }
    }


    void NewtonRigidBody::MarkDirty(bool dirty)
    {
        needsRebuilt_ = dirty;
    }

    void NewtonRigidBody::MarkInternalTransformDirty(bool dirty)
    {
        transformDirty_ = dirty;
    }

    bool NewtonRigidBody::GetInternalTransformDirty()
    {
        return transformDirty_;
    }


    void NewtonRigidBody::OnSetEnabled()
    {
        MarkDirty(true);
    }

    void NewtonRigidBody::freeBody()
    {
        if (newtonBody_ != nullptr) {
            NewtonDestroyBody(newtonBody_);
            newtonBody_ = nullptr;
        }

        //also free the compound collision if there is one
        if (compoundCollision_)
        {
            NewtonDestroyCollision(compoundCollision_);
            compoundCollision_ = nullptr;
        }
    }



    void NewtonRigidBody::reBuildBody()
    {
        URHO3D_PROFILE_FUNCTION();
        freeBody();

        if (!IsEnabledEffective())
            return;



        GSS<VisualDebugger>()->AddOrb(GetNode()->GetWorldPosition(), 0.5f, Color::RED);
        //evaluate child nodes (+this node) and see if there are more collision shapes - if so create a compound collision.
        PODVector<NewtonCollisionShape*> childCollisionShapes;
        node_->GetDerivedComponents<NewtonCollisionShape>(childCollisionShapes, true, true);//includes this node.
        
        PODVector<NewtonCollisionShape*> filteredList;
        //if scene root body - filter out shapes with no root rigid body (except the scene root of course)
        if (sceneRootBodyMode_)
        {
            for (NewtonCollisionShape* col : childCollisionShapes)
            {
                PODVector<NewtonRigidBody*> parentRigBodies;
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



                if (compoundNeeded)
                    usedCollision = NewtonCollisionCreateInstance(curNewtCollision);
                else
                    usedCollision = curNewtCollision;

                Vector3 savedScale = colComp->GetNode()->GetScale();
                colComp->GetNode()->SetScale(1.0f);//temp set node scale to 1 so we can use nice functions below.

                Matrix3x4 uMat = colComp->GetNode()->LocalToWorld(colComp->getInternalOffsetMatrix()*colComp->GetOffsetMatrix());
                Matrix3x4 localTransformNoScale = node_->WorldToLocal(uMat);
                dMatrix localTransform = UrhoToNewton(localTransformNoScale);

                NewtonCollisionSetMatrix(usedCollision, &localTransform[0][0]);//set the collision matrix with translation and rotation data only.

                colComp->GetNode()->SetScale(savedScale);//restore node scale.

                if (inheritCollisionNodeScales_)
                {
                    
                    Vector3 scale = colComp->GetNode()->GetScale();


                    scale = (colComp->internalRotOffset_*colComp->GetRotationOffset()).Inverse()*scale;


                    NewtonCollisionSetScale(usedCollision, scale.x_, scale.y_, scale.z_);//then scale.
                }
                accumMass += colComp->GetVolume()*1.0f;

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

            mass_ = accumMass * massScale_;
            if (sceneRootBodyMode_)
                mass_ = 0;

            NewtonBodySetMassProperties(newtonBody_, mass_, resolvedCollision);

            NewtonBodySetUserData(newtonBody_, (void*)this);

            NewtonBodySetContinuousCollisionMode(newtonBody_, continuousCollision_);

            //ensure newton damping is 0 because we apply our own as a force.
            NewtonBodySetLinearDamping(newtonBody_, linearDampeningInternal_);
            NewtonBodySetAngularDamping(newtonBody_, &UrhoToNewton(angularDampeningInternal_)[0]);



            //assign callbacks
            NewtonBodySetForceAndTorqueCallback(newtonBody_, Newton_ApplyForceAndTorqueCallback);
            NewtonBodySetTransformCallback(newtonBody_, Newton_SetTransformCallback); 
            NewtonBodySetDestructorCallback(newtonBody_, Newton_DestroyBodyCallback);
        }
    }





    void NewtonRigidBody::bakeForceAndTorque()
    {

    }

    void NewtonRigidBody::OnNodeSet(Node* node)
    {
        if (node)
        {

            //Auto-create a physics world on the scene if it does not yet exist.
            physicsWorld_ = WeakPtr<UrhoNewtonPhysicsWorld>(GetScene()->GetOrCreateComponent<UrhoNewtonPhysicsWorld>());

            physicsWorld_->addRigidBody(this);

            node->AddListener(this);

            SubscribeToEvent(node, E_NODETRANSFORMCHANGE, URHO3D_HANDLER(NewtonRigidBody, HandleNodeTransformChange));
        }
        else
        {
            if (physicsWorld_)
                physicsWorld_->removeRigidBody(this);

            freeBody();
            UnsubscribeFromEvent(E_NODETRANSFORMCHANGE);
        }

    }

    void NewtonRigidBody::OnSceneSet(Scene* scene)
    {

    }
    void NewtonRigidBody::HandleNodeAdded(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeAdded::P_NODE].GetPtr());
        Node* newParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());

        if (node == node_)
        {
            OnPhysicsNodeAdded(node);
        }
    }

    void NewtonRigidBody::HandleNodeRemoved(StringHash event, VariantMap& eventData)
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


   

    void NewtonRigidBody::HandleNodeTransformChange(StringHash event, VariantMap& eventData)
    {
        NewtonRigidBody* parentRigidBody = node_->GetParentComponent<NewtonRigidBody>(true);
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
            PODVector<NewtonRigidBody*> parentRigBodies;
            GetRootRigidBodies(parentRigBodies, node_, false);
            parentRigBodies.Back()->MarkDirty();

        }
    }



    void NewtonRigidBody::applyDefferedActions()
    {
        if (nextLinearVelocityNeeded_)
        {
            if (newtonBody_)
            {
                //AddImpulse()
                AddImpulse(Vector3::ZERO, nextLinearVelocity_);
                //NewtonBodySetVelocity(newtonBody_, &UrhoToNewton(nextLinearVelocity_)[0]);
            }
            nextLinearVelocityNeeded_ = false;
        }
    }

    void NewtonRigidBody::OnNodeSetEnabled(Node* node)
    {
        if (node == node_)
        {
            MarkDirty(true);
        }
    }

    void NewtonRigidBody::AddWorldForce(const Vector3& force)
    {
        AddWorldForce(force, Vector3::ZERO);
    }

    void NewtonRigidBody::AddWorldForce(const Vector3& force, const Vector3& localPosition)
    {
        netForce_ += force;
        netTorque_ += localPosition.CrossProduct(node_->WorldToLocal(force));
        bakeForceAndTorque();
    }

    void NewtonRigidBody::AddWorldTorque(const Vector3& torque)
    {
        netTorque_ += torque;
        bakeForceAndTorque();
    }

    void NewtonRigidBody::AddLocalForce(const Vector3& force)
    {
        AddWorldForce(node_->LocalToWorld(force));
    }

    void NewtonRigidBody::AddLocalForce(const Vector3& force, const Vector3& localPosition)
    {
        AddWorldForce(node_->LocalToWorld(force), localPosition);
    }

    void NewtonRigidBody::AddLocalTorque(const Vector3& torque)
    {
        AddWorldTorque(node_->LocalToWorld(torque));
    }

    void NewtonRigidBody::ResetForces()
    {
        netForce_ = Vector3(0, 0, 0);
        netTorque_ = Vector3(0, 0, 0);
        bakeForceAndTorque();
    }

    void NewtonRigidBody::AddImpulse(const Vector3& localPosition, const Vector3& targetVelocity)
    {
        if(newtonBody_)
            NewtonBodyAddImpulse(newtonBody_, &UrhoToNewton(targetVelocity)[0], &UrhoToNewton(node_->LocalToWorld(localPosition))[0], GSS<Engine>()->GetUpdateTimeGoalMs()*0.001f);

        
    }

    Vector3 NewtonRigidBody::GetNetForce()
    {
        return netForce_;
    }


    Urho3D::Vector3 NewtonRigidBody::GetNetTorque()
    {
        return netTorque_;
    }

    NewtonCollision* NewtonRigidBody::GetEffectiveNewtonCollision()
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

    Vector3 NewtonRigidBody::GetVelocity()
    {
        if (newtonBody_) {
            dVector vel;
            NewtonBodyGetVelocity(newtonBody_, &vel[0]);
            return NewtonToUrhoVec3(vel);
        }
        else
            return Vector3::ZERO;
    }

    Vector3 NewtonRigidBody::GetAngularVelocity(TransformSpace space)
    {
        if (newtonBody_) {
            dVector vel;
            NewtonBodyGetOmega(newtonBody_, &vel[0]);

            if (space == TS_WORLD)
            {
                return NewtonToUrhoVec3(vel);
            }
            else if(space == TS_LOCAL)
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


    Vector3 NewtonRigidBody::GetAcceleration()
    {
        if (newtonBody_) {
            dVector vel;
            NewtonBodyGetAcceleration(newtonBody_, &vel[0]);
            return NewtonToUrhoVec3(vel);
        }
        else
            return Vector3::ZERO;
    }

    Vector3 NewtonRigidBody::GetCenterOfMassPosition()
    {
        dVector pos;
        NewtonBodyGetPosition(newtonBody_, &pos[0]);
        return NewtonToUrhoVec3(pos);
    }

    Quaternion NewtonRigidBody::GetCenterOfMassRotation()
    {
        dQuaternion quat;
        NewtonBodyGetRotation(newtonBody_, &quat.m_q0);
        return NewtonToUrhoQuat(quat);
    }

    void NewtonRigidBody::GetConnectedContraints(PODVector<NewtonConstraint*>& contraints)
    {
        contraints.Clear();
        for (auto i = connectedConstraints_.Begin(); i != connectedConstraints_.End(); ++i)
        {
            contraints += *i;
        }
    }

    void NewtonRigidBody::ApplyTransform()
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


    void NewtonRigidBody::GetForceAndTorque(dVector& force, dVector& torque)
    {
        URHO3D_PROFILE("GetForceAndTorque");


        Vector3 gravityForce = GetScene()->GetComponent<UrhoNewtonPhysicsWorld>()->GetGravity() * mass_;


        //basic damping forces (clamp to some reasonable values)
        float linearDampingClamped = Urho3D::Clamp<float>(linearDampening_, 0.0f, 0.3f);
        Vector3 angularDampingClamped = Urho3D::VectorClamp(angularDampening_, Vector3::ZERO, Vector3::ONE * 0.3f);


        //basic velocity damping forces
        Vector3 velocity = GetVelocity();
        Vector3 linearDampingForce = -velocity.Normalized()*(velocity.LengthSquared())*linearDampingClamped * mass_;

        //basic angular damping forces
        Vector3 angularVelocity = GetAngularVelocity();
        Vector3 angularDampingTorque = Vector3::ZERO;// -angularVelocity.Normalized()*(angularVelocity.LengthSquared())*angularDampingClamped * mass_;

        force = UrhoToNewton((linearDampingForce + gravityForce + netForce_));
        torque = UrhoToNewton(angularDampingTorque + netTorque_);

    }

}
