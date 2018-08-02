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
#include "dgQuaternion.h"
#include "Scene/SceneEvents.h"
#include "Engine/Engine.h"


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
        massScale_ = massDensityScale;
        MarkDirty(true);
    }

    void NewtonRigidBody::SetFriction(float friction)
    {
    }

    void NewtonRigidBody::SetLinearVelocity(const Vector3& velocity)
    {
        if(newtonBody_)
            NewtonBodySetVelocity(newtonBody_, &UrhoToNewton(velocity)[0]);
    }

    void NewtonRigidBody::SetContinuousCollision(bool sweptCollision)
    {
        continuousCollision_ = sweptCollision;

    }

    void NewtonRigidBody::DrawDebugGeometry(DebugRenderer* debug, bool depthTest, bool showAABB /*= true*/, bool showCollisionMesh /*= true*/, bool showCenterOfMass /*= true*/, bool showContactForces /*= true*/)
    {
        Component::DrawDebugGeometry(debug, depthTest);
        if (newtonBody_) {
            if (showAABB) NewtonBodyDebugDrawAABB(newtonBody_, debug, depthTest);
            if (showCollisionMesh) NewtonBodyDebugShowCollision(newtonBody_, debug, depthTest);
            if (showCenterOfMass) NewtonBodyDebugDrawCenterOfMass(newtonBody_, debug, depthTest);
            if (showContactForces)  NewtonBodyDebugContactForces(newtonBody_, 0., debug, depthTest);
        }
    }


    void NewtonRigidBody::MarkDirty(bool dirty)
    {
        needsRebuilt_ = dirty;
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
    void NewtonRigidBody::reBuildBodyParent()
    {
        //determine if there is a parent rigid body and if there is we do not want to create a new body on this - we want to form a compound collision on the parent
        Node* curNode = node_;
        Node* parentNodeWithRigidBody = nullptr;
        while (curNode != nullptr) {
            curNode = curNode->GetParent();
            if (curNode && curNode->GetComponent<NewtonRigidBody>()) {
                parentNodeWithRigidBody = curNode;
                break;
            }
        }
        if (parentNodeWithRigidBody != nullptr)
        {
            MarkDirty(false);//mark as clean because we know a rigid body on a higher level is going to be the real one.
            parentNodeWithRigidBody->GetComponent<NewtonRigidBody>()->reBuildBodyParent();
            return;
        }
        reBuildBody();
        MarkDirty(false);//restore our dirtyness
    }


    void NewtonRigidBody::reBuildBody()
    {

        freeBody();

        //evaluate child nodes (+this node) and see if there are more collision shapes - if so create a compound collision.
        PODVector<Node*> nodesWithCollision;
        PODVector<NewtonCollisionShape*> childCollisionShapes;
        node_->GetDerivedComponents<NewtonCollisionShape>(childCollisionShapes, true, true);

        for (NewtonCollisionShape* shp : childCollisionShapes)
            nodesWithCollision += shp->GetNode();

        if(node_->GetDerivedComponent<NewtonCollisionShape>())
            nodesWithCollision += node_;




        if (nodesWithCollision.Size())
        {
            NewtonCollision* resolvedCollision = nullptr;
            if (compoundCollision_) {
                NewtonDestroyCollision(compoundCollision_);
                compoundCollision_ = nullptr;
            }
            bool compoundNeeded = (nodesWithCollision.Size() > 1);

            if (compoundNeeded) {
                compoundCollision_ = NewtonCreateCompoundCollision(physicsWorld_->GetNewtonWorld(), 0);
                NewtonCompoundCollisionBeginAddRemove(compoundCollision_);
            }

            float accumMass = 0.0f;
            for (Node* curNode : nodesWithCollision)
            {
                NewtonCollisionShape* colComp = curNode->GetDerivedComponent<NewtonCollisionShape>();

                //make sure the collision is built with latest.
                if (colComp->shapeNeedsRebuilt_)
                    colComp->reEvaluateCollision();

                NewtonCollision* curNewtCollision = colComp->GetNewtonCollision();
                NewtonCollision* usedCollision = nullptr;

                if (compoundNeeded)
                    usedCollision = NewtonCollisionCreateInstance(curNewtCollision);
                else
                    usedCollision = curNewtCollision;

                //Matrix4 uMat = node_->WorldToLocal(curNode->GetWorldTransform());
                Matrix3x4 uMat = curNode->GetWorldTransform();
                //uMat.SetTranslation(curNode->GetWorldPosition());
                //uMat.SetRotation(curNode->GetWorldRotation().RotationMatrix());
                //uMat.SetScale(1.0f);
                dMatrix localTransform = UrhoToNewton(node_->WorldToLocal(uMat));

    
                
                NewtonCollisionSetScale(usedCollision, curNode->GetScale().x_, curNode->GetScale().y_, curNode->GetScale().z_);
                NewtonCollisionSetMatrix(usedCollision, &localTransform[0][0]);
                accumMass += colComp->GetVolume()*1.0f;


                if (compoundNeeded) {
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

            if (!newtonBody_)
                newtonBody_ = NewtonCreateDynamicBody(physicsWorld_->GetNewtonWorld(), resolvedCollision, &mat[0][0]);


            NewtonBodySetCollision(newtonBody_, resolvedCollision);

            mass_ = accumMass * massScale_;
            NewtonBodySetMassProperties(newtonBody_, mass_, resolvedCollision);
            
            NewtonBodySetUserData(newtonBody_, (void*)this);

            NewtonBodySetContinuousCollisionMode(newtonBody_, continuousCollision_);


            //assign callbacks
            NewtonBodySetForceAndTorqueCallback(newtonBody_, Newton_ApplyForceAndTorqueCallback);
            //NewtonBodySetTransformCallback(newtonBody_, Newton_SetTransformCallback); //not really needed since we pole for the transform at a specific time.
            NewtonBodySetDestructorCallback(newtonBody_, Newton_DestroyBodyCallback);

            bakeForceAndTorque();
        }

    }





    void NewtonRigidBody::bakeForceAndTorque()
    {
        Vector3 gravityForce = GetScene()->GetComponent<UrhoNewtonPhysicsWorld>()->GetGravity() * mass_;

        //Quaternion worldOrientation = node_->GetWorldRotation();
        //Vector3 netForceWrldSpc = worldOrientation * netForce_;
        //Vector3 netTorqueWrldSpc = worldOrientation * netTorque_;

        netForceNewton_ = UrhoToNewton((gravityForce + netForce_));
        netTorqueNewton_ = UrhoToNewton(netTorque_);
    }

    void NewtonRigidBody::OnNodeSet(Node* node)
    {
        if (node)
        {

            if (node == GetScene())
                URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

            //Auto-create a physics world on the scene if it does not yet exist.
            physicsWorld_ = WeakPtr<UrhoNewtonPhysicsWorld>(GetScene()->GetOrCreateComponent<UrhoNewtonPhysicsWorld>());
            physicsWorld_->addRigidBody(this);
            MarkDirty(true);

            if (colShape_)
                colShape_->updateReferenceToRigidBody();


            SubscribeToEvent(node, E_NODETRANSFORMCHANGE, URHO3D_HANDLER(NewtonRigidBody, HandleNodeTransformChange));
        }
        else
        {
            if (physicsWorld_)
                physicsWorld_->removeRigidBody(this);

            freeBody();
            if (colShape_)
                colShape_->updateReferenceToRigidBody();

            UnsubscribeFromEvent(E_NODETRANSFORMCHANGE);
        }

    }

    void NewtonRigidBody::OnSceneSet(Scene* scene)
    {

    }


    void NewtonRigidBody::HandleNodeAdded(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeAdded::P_NODE].GetPtr());

        //if the node has been added to a parent.
        if (node == node_)
        {
            //markt the rigid body dirty
            if (node->HasComponent<NewtonRigidBody>())
            {
                node->GetComponent<NewtonRigidBody>()->MarkDirty();
            }

            //mark its old parent dirty as well.
            if (oldNodeParent_)
            {
                NewtonRigidBody* oldParentRigidBody = oldNodeParent_->GetComponent<NewtonRigidBody>();
                if (!oldParentRigidBody)
                    oldParentRigidBody = oldNodeParent_->GetParentComponent<NewtonRigidBody>(true);

                if (oldParentRigidBody)
                    oldParentRigidBody->MarkDirty();
            }
            oldNodeParent_ = nullptr;
        }
    }

    void NewtonRigidBody::HandleNodeRemoved(StringHash event, VariantMap& eventData)
    {
        Node* node = static_cast<Node*>(eventData[NodeRemoved::P_NODE].GetPtr());
        if (node == node_)
        {
            Node* oldParent = static_cast<Node*>(eventData[NodeRemoved::P_PARENT].GetPtr());
            oldNodeParent_ = oldParent;
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
            if (parentRigidBody) {
                parentRigidBody->MarkDirty();
            }

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
            NewtonBodyAddImpulse(newtonBody_, &UrhoToNewton(targetVelocity)[0], &UrhoToNewton(node_->LocalToWorld(localPosition))[0], GSS<Engine>()->GetUpdateTimeGoalMs());

        
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

    Vector3 NewtonRigidBody::GetCenterOfMassPosition()
    {
        dVector pos;
        NewtonBodyGetPosition(newtonBody_, &pos[0]);
        return NewtonToUrhoVec3(pos);
    }

    Quaternion NewtonRigidBody::GetCenterOfMassRotation()
    {
        dgQuaternion quat;
        NewtonBodyGetRotation(newtonBody_, &quat[0]);
        return NewtonToUrhoQuat(quat);
    }

    void NewtonRigidBody::ApplyTransform()
    {
        if (!newtonBody_)
            return;

        dVector pos;
        dgQuaternion quat;
        NewtonBodyGetPosition(newtonBody_, &pos[0]);
        NewtonBodyGetRotation(newtonBody_, &quat[0]);

        bool enableTEvents = node_->GetEnableTransformEvents();
        if(enableTEvents)
            node_->SetEnableTransformEvents(false);


        node_->SetWorldTransform(NewtonToUrhoVec3(pos), NewtonToUrhoQuat(quat));

        node_->SetEnableTransformEvents(enableTEvents);
    }


    void NewtonRigidBody::GetBakedForceAndTorque(dVector& force, dVector& torque)
    {
        force = netForceNewton_;
        torque = netTorqueNewton_;
    }

}
