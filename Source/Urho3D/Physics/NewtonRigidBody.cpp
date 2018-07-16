#include "../Physics/UrhoNewtonPhysicsWorld.h"
#include "../Physics/NewtonCollisionShape.h"
#include "../Physics/NewtonRigidBody.h"
#include "../Core/Context.h"
#include "IO/Log.h"
#include "Scene/Scene.h"
#include "Scene/Node.h"
#include "dMatrix.h"
#include "Newton.h"
#include "NewtonDebugDrawing.h"
#include "UrhoNewtonConversions.h"
#include "dgQuaternion.h"


namespace Urho3D {






    NewtonRigidBody::NewtonRigidBody(Context* context) : Component(context)
    {

    }

    NewtonRigidBody::~NewtonRigidBody()
    {

    }

    void NewtonRigidBody::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonRigidBody>();

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



    void NewtonRigidBody::SetMass(float mass)
    {
        mass_ = mass;

        reEvaluateBody();
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


    void NewtonRigidBody::freeBody()
    {
        if (newtonBody_ != nullptr) {
            NewtonDestroyBody(newtonBody_);
            newtonBody_ = nullptr;
        }
    }



    void NewtonRigidBody::reEvaluateBody()
    {
        URHO3D_LOGINFO("reEvaluating Body.");


        colShape_ = node_->GetComponent<NewtonCollisionShape>();

        //determine if there is a parent rigid body and if there is we do not want to create a new body - we want to form a compound collision on the parent
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
            freeBody();
            parentNodeWithRigidBody->GetComponent<NewtonRigidBody>()->reEvaluateBody();
            return;
        }


        if (colShape_ &&
            (colShape_->GetNewtonCollision() != nullptr) ) {

            Matrix4 transform;
            transform.SetTranslation(node_->GetWorldPosition());
            transform.SetRotation(node_->GetWorldRotation().RotationMatrix());
           
            dMatrix mat = UrhoToNewton(transform);

            NewtonWorld* newtWorld = GetScene()->GetComponent<UrhoNewtonPhysicsWorld>()->GetNewtonWorld();



            //evaluate child nodes and see if there are more collision shapes - if so create a compound collision.
            PODVector<Node*> children;
            node_->GetChildrenWithComponent<NewtonCollisionShape>(children, true);

            NewtonCollision* resolvedCollision = nullptr;
            if (compoundCollision_) {
                NewtonDestroyCollision(compoundCollision_);
                compoundCollision_ = nullptr;
            }

            if (children.Size())
            {
                compoundCollision_ = NewtonCreateCompoundCollision(newtWorld, 0);

                //...
                NewtonCompoundCollisionBeginAddRemove(compoundCollision_);

                for (Node* childNode : children)
                {
                    if (childNode->GetComponent<NewtonRigidBody>()) {
                        mass_ += childNode->GetComponent<NewtonRigidBody>()->mass_;
                    }


                    NewtonCollision* childCollision = childNode->GetComponent<NewtonCollisionShape>()->GetNewtonCollision();
                    NewtonCollision* tempTransformedChildCollision = NewtonCollisionCreateInstance(childCollision);

                    dMatrix localTransform = UrhoToNewton(node_->WorldToLocal(childNode->GetWorldTransform()));



                    NewtonCollisionSetMatrix(tempTransformedChildCollision, &localTransform[0][0]);

                    NewtonCompoundCollisionAddSubCollision(compoundCollision_, tempTransformedChildCollision);

                    NewtonDestroyCollision(tempTransformedChildCollision);
                }

                //finally add this collision
                NewtonCompoundCollisionAddSubCollision(compoundCollision_, colShape_->GetNewtonCollision());


                NewtonCompoundCollisionEndAddRemove(compoundCollision_);


                resolvedCollision = compoundCollision_;

            }
            else
            {
                resolvedCollision = colShape_->GetNewtonCollision();
            }




            if(!newtonBody_)
                newtonBody_ = NewtonCreateDynamicBody(newtWorld, resolvedCollision, &mat[0][0]);




            NewtonBodySetCollision(newtonBody_, resolvedCollision);

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

        Quaternion worldOrientation = GetNode()->GetWorldRotation();
        Vector3 netForceWrldSpc = worldOrientation * netForce_;
        Vector3 netTorqueWrldSpc = worldOrientation * netTorque_;

        netForceNewton_ = UrhoToNewton((gravityForce + netForceWrldSpc));
        netTorqueNewton_ = UrhoToNewton(netTorqueWrldSpc);
    }

    void NewtonRigidBody::OnNodeSet(Node* node)
    {
        if (node)
        {
            reEvaluateBody();
            if(colShape_)
                colShape_->updateReferenceToRigidBody();
        }
        else
        {
            freeBody();
            if (colShape_)
                colShape_->updateReferenceToRigidBody();
        }
    }

    void NewtonRigidBody::OnSceneSet(Scene* scene)
    {
        if (scene)
        {
            if (scene == node_)
                URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

            //Auto-create a physics world on the scene if it does not yet exist.
            physicsWorld_ = WeakPtr<UrhoNewtonPhysicsWorld>(scene->GetOrCreateComponent<UrhoNewtonPhysicsWorld>());
            physicsWorld_->addRigidBody(this);
        }
        else
        {

            if (physicsWorld_)
                physicsWorld_->removeRigidBody(this);

        }
    }

    void NewtonRigidBody::AddForce(const Vector3& force)
    {
        AddForce(force, Vector3(0, 0, 0));
    }

    void NewtonRigidBody::AddForce(const Vector3& force, const Vector3& position)
    {
        netForce_ += force;
        netTorque_ += position.CrossProduct(force);
        bakeForceAndTorque();
    }

    void NewtonRigidBody::ResetForces()
    {
        netForce_ = Vector3(0, 0, 0);
        netTorque_ = Vector3(0, 0, 0);
        bakeForceAndTorque();
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

    void NewtonRigidBody::ApplyTransform()
    {
        if (!newtonBody_)
            return;

        dVector pos;
        dgQuaternion quat;
        NewtonBodyGetPosition(newtonBody_, &pos[0]);
        NewtonBodyGetRotation(newtonBody_, &quat[0]);


        //Vector3 positionalOffset;
        //if (compoundCollision_) {
        //    dVector localCenterOfMass;
        //    NewtonBodyGetCentreOfMass(newtonBody_, &localCenterOfMass[0]);
        //    positionalOffset = -NewtonToUrhoVec3(localCenterOfMass);

        //}

        //node_->SetWorldTransform(NewtonToUrhoVec3(pos), Quaternion());
        node_->SetWorldTransform(NewtonToUrhoVec3(pos), NewtonToUrhoQuat(quat));
    }


    void NewtonRigidBody::GetBakedForceAndTorque(dVector& force, dVector& torque)
    {
        force = netForceNewton_;
        torque = netTorqueNewton_;
    }

}
