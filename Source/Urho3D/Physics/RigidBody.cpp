#include "../Physics/PhysicsWorld.h"
#include "../Physics/CollisionShape.h"
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
#include "Constraint.h"
#include "PhysicsMaterial.h"
#include "Resource/ResourceCache.h"


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

        URHO3D_MIXED_ACCESSOR_ATTRIBUTE("PhysicsMaterial", GetPhysMaterialAttr, SetPhysMaterialAttr, ResourceRef, ResourceRef(PhysicsMaterial::GetTypeStatic()), AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("MassScale", GetMassScale, SetMassScale, float, 1.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Linear Velocity", GetLinearVelocity, SetLinearVelocity, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Angular Velocity", GetAngularVelocity, SetAngularVelocity, Vector3, Vector3::ZERO, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Inherit Collision Node Scales", GetInheritNodeScale, SetInheritNodeScale, bool, true, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Continuous Collision", GetContinuousCollision, SetContinuousCollision, bool, false, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Linear Damping", GetLinearDamping, SetLinearDamping, float, 0.0f, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Angular Damping", GetAngularDamping, SetAngularDamping, float, 0.0f, AM_DEFAULT);
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
    void RigidBody::SetPhysicsMaterial(PhysicsMaterial* material)
    {
        if (material != physicsMaterial_) {
            physicsMaterial_ = material;

            //add the physics material to the newton world if it is not there already.
            physicsWorld_->addPhysicsMaterial(material);

            if (newtonBody_)
            {
                NewtonBodySetMaterialGroupID(newtonBody_, physicsMaterial_->newtonGroupId);
            }
            else {
                MarkDirty(true);
            }

        }
    }
    void RigidBody::SetPhysMaterialAttr(const ResourceRef& value)
    {
        auto* cache = GetSubsystem<ResourceCache>();
        SetPhysicsMaterial(cache->GetResource<PhysicsMaterial>(value.name_));
    }


    ResourceRef RigidBody::GetPhysMaterialAttr() const
    {
        return GetResourceRef(physicsMaterial_, PhysicsMaterial::GetTypeStatic());
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
        dampingFactor = Urho3D::Clamp<float>(dampingFactor, 0.0f, dampingFactor);

        if (linearDampening_ != dampingFactor) {
            linearDampening_ = dampingFactor;
        }
    }

    void RigidBody::SetAngularDamping(float angularDamping)
    {
        angularDamping = Urho3D::Clamp(angularDamping, 0.0f, angularDamping);

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
            else
            {
                MarkDirty();
            }
        }
    }

    void RigidBody::SetInternalAngularDamping(float angularDamping)
    {
        angularDampeningInternal_ = Vector3(angularDamping, angularDamping, angularDamping);
        if (newtonBody_)
        {
            NewtonBodySetAngularDamping(newtonBody_, &UrhoToNewton(angularDampeningInternal_)[0]);
        }
        else
        {
            MarkDirty();
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
            }
        }
    }

    void RigidBody::Activate()
    {
        if (newtonBody_)
        {
            NewtonBodySetSleepState(newtonBody_, false);
        }
        else
        {
            nextSleepStateNeeded_ = true;
            nextSleepState_ = false;
        }
    }

    void RigidBody::DeActivate()
    {
        if (newtonBody_)
        {
            NewtonBodySetSleepState(newtonBody_, true);
        }
        else
        {
            nextSleepStateNeeded_ = true;
            nextSleepState_ = true;
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
        if (IsEnabledEffective()) {
            MarkDirty(true);//rebuild
        }
        else
        {
            freeBody();
        }
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
        PODVector<CollisionShape*> childCollisionShapes;

        GetAloneCollisionShapes(childCollisionShapes, node_, true);


        PODVector<CollisionShape*> filteredList;

        //filter out shapes that are not enabled.
        for (CollisionShape* col : childCollisionShapes)
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

            for (CollisionShape* colComp : childCollisionShapes)
            {
                NewtonCollision* curNewtCollision = colComp->GetNewtonCollision();
                NewtonCollision* usedCollision = nullptr;


                if (compoundNeeded)
                    usedCollision = NewtonCollisionCreateInstance(curNewtCollision);
                else
                    usedCollision = curNewtCollision;



                Matrix3x4 colWorldTransformNoScale = Matrix3x4(colComp->GetNode()->GetWorldPosition(), colComp->GetNode()->GetWorldRotation(), 1.0f);
                Matrix3x4 thisWorldTransformNoScale = Matrix3x4(node_->GetWorldPosition(), node_->GetWorldRotation(), 1.0f);
                Matrix3x4 thisLocalTransformNoScale = Matrix3x4(node_->GetPosition(), node_->GetRotation(), 1.0f);


                Matrix3x4 colLocalOffsetTransform = colComp->GetOffsetMatrix();

                Matrix3x4 colWorldOffset = colWorldTransformNoScale * colLocalOffsetTransform;
                Matrix3x4 colLocalToThisNode = thisWorldTransformNoScale.Inverse()*colWorldOffset;



                dMatrix localTransform = UrhoToNewton(colLocalToThisNode);
                NewtonCollisionSetMatrix(usedCollision, &localTransform[0][0]);//set the collision matrix with translation and rotation data only.


                    
                Vector3 scale;
                if (inheritCollisionNodeScales_)
                {
                    scale = colComp->GetNode()->GetWorldScale();
                }
                Vector3 shapeScale = colComp->GetScaleFactor();

                scale = (colComp->GetRotationOffset()).Inverse()*scale*shapeScale;


                NewtonCollisionSetScale(usedCollision, scale.x_, scale.y_, scale.z_);//then scale.

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

            //set auto sleep mode.
            NewtonBodySetAutoSleep(newtonBody_, autoSleep_);


            //assign material
            if(physicsMaterial_)
                NewtonBodySetMaterialGroupID(newtonBody_, physicsMaterial_->newtonGroupId);

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
            physicsWorld_ = WeakPtr<PhysicsWorld>(GetScene()->GetOrCreateComponent<PhysicsWorld>());

            physicsWorld_->addRigidBody(this);

            node->AddListener(this);

            //assign the default physics material.
            if (physicsMaterial_ == nullptr)
                SetPhysicsMaterial(physicsWorld_->defaultPhysicsMaterial_);

            SubscribeToEvent(node, E_NODETRANSFORMCHANGE, URHO3D_HANDLER(RigidBody, HandleNodeTransformChange));

            prevNode_ = node;
        }
        else
        {

            if (physicsWorld_)
                physicsWorld_->removeRigidBody(this);

            //remove any connected constraints.
            for (Constraint* constraint : connectedConstraints_) {
                constraint->Remove();
            }



            freeBody();
            UnsubscribeFromEvent(E_NODETRANSFORMCHANGE);

            prevNode_ = nullptr;
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
            RebuildPhysicsNodeTree(node);
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
                RebuildPhysicsNodeTree(oldParent);
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
            Matrix3x4 mat(eventData[NodeTransformChange::P_NEW_POSITION].GetVector3(),
                eventData[NodeTransformChange::P_NEW_ORIENTATION].GetQuaternion(),
                Vector3::ONE);

            //if the scale changed - we need to rebuild - else just set the transform directly.
            if (eventData[NodeTransformChange::P_NEW_SCALE].GetVector3() != eventData[NodeTransformChange::P_OLD_SCALE].GetVector3())
                MarkDirty(true);
            else
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
        if (nextSleepStateNeeded_) {

            if (newtonBody_)
            {
                NewtonBodySetSleepState(newtonBody_, nextSleepState_);
            }
            nextSleepStateNeeded_ = false;
        }


    }

    void RigidBody::OnNodeSetEnabled(Node* node)
    {
        if (node == node_)
        {
            if (IsEnabledEffective()) {
                MarkDirty(true);//rebuild
            }
            else
            {
                freeBody();
            }
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
            if (node_->GetComponent<CollisionShape>()) {
                return node_->GetComponent<CollisionShape>()->GetNewtonCollision();
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

    void RigidBody::GetConnectedContraints(PODVector<Constraint*>& contraints)
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

        //Vector3 localScale = node_->GetScale();
        //URHO3D_LOGINFO(String()
        node_->SetWorldTransform(NewtonToUrhoVec3(pos), NewtonToUrhoQuat(quat));
        //node_->SetScale(localScale);//not sure why I have to restore this - without it, the worldscale of the node approaches infinity over time (some precision error in the matrix code?)


        node_->SetEnableTransformEvents(enableTEvents);
    }


    void RigidBody::GetForceAndTorque(Vector3& force, Vector3& torque)
    {
        URHO3D_PROFILE("GetForceAndTorque");

        //basic velocity damping forces
        Vector3 velocity = GetLinearVelocity(TS_WORLD);
        Vector3 linearDampingForce = -velocity.Normalized()*(velocity.LengthSquared())*linearDampening_ * mass_;

        if (linearDampingForce.Length() <= M_EPSILON)
            linearDampingForce = Vector3::ZERO;


        //basic angular damping forces
        Vector3 angularVelocity = GetAngularVelocity(TS_WORLD);
        Vector3 angularDampingTorque = -angularVelocity.Normalized()*(angularVelocity.LengthSquared())*angularDampening_ * mass_;

        if (angularVelocity.Length() <= M_EPSILON)
            angularDampingTorque = Vector3::ZERO;


        force = linearDampingForce + netForce_;
        torque = angularDampingTorque + netTorque_;

    }

}
