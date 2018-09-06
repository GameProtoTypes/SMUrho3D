#include "PhysicsWorld.h"
#include "Core/Profiler.h"
#include "RigidBody.h"
#include "Scene/Component.h"
#include "Scene/Scene.h"
#include "UrhoNewtonConversions.h"
#include "PhysicsMaterial.h"
#include "IO/Log.h"
#include "Newton.h"


namespace Urho3D {

    void Newton_ApplyForceAndTorqueCallback(const NewtonBody* body, dFloat timestep, int threadIndex)
    {
        URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).CString());
        URHO3D_PROFILE_FUNCTION()


            Vector3 netForce;
        Vector3 netTorque;
        RigidBody* rigidBodyComp = nullptr;

        rigidBodyComp = static_cast<RigidBody*>(NewtonBodyGetUserData(body));


        rigidBodyComp->GetForceAndTorque(netForce, netTorque);


        Vector3 gravityForce;
        if (rigidBodyComp->GetScene())//on scene destruction sometimes this is null so check...
            gravityForce = rigidBodyComp->GetScene()->GetComponent<PhysicsWorld>()->GetGravity() * rigidBodyComp->GetEffectiveMass();

        netForce += gravityForce;

        NewtonBodySetForce(body, &UrhoToNewton(netForce)[0]);
        NewtonBodySetTorque(body, &UrhoToNewton(netTorque)[0]);
    }


    void Newton_SetTransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
    {
        RigidBody* rigBody = static_cast<RigidBody*>(NewtonBodyGetUserData(body));
        rigBody->MarkInternalTransformDirty();
    }


    void Newton_DestroyBodyCallback(const NewtonBody* body)
    {

    }

    unsigned Newton_WorldRayPrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
    {
        ///no filtering right now.
        return 1;///?
    }








    void Newton_ProcessContactsCallback(const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
    {
        URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).CString());
        URHO3D_PROFILE_FUNCTION();



        const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
        const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

        RigidBody* rigBody0 = static_cast<RigidBody*>(NewtonBodyGetUserData(body0));
        RigidBody* rigBody1 = static_cast<RigidBody*>(NewtonBodyGetUserData(body1));

        PhysicsWorld* physicsWorld = rigBody0->GetPhysicsWorld();


        for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
            NewtonMaterial* const material = NewtonContactGetMaterial(contact);

            //extract the urho pairData from the newton pair.
            PhysicsMaterialContactPair* pairData = static_cast<PhysicsMaterialContactPair*>(NewtonMaterialGetMaterialPairUserData(material));
            
            NewtonMaterialSetContactFrictionCoef(material, pairData->staticFrictionCoef_, pairData->kineticFrictionCoef_, 0);
            NewtonMaterialSetContactElasticity(material, pairData->elasticity_);
            NewtonMaterialSetContactSoftness(material, pairData->softness_);
        }


        //insert a struct into the physicsWorld body contact state map

        //causes threading conflict.
        //physicsWorld->TouchBodyContactMap(rigBody0, rigBody1);



    }






    int Newton_AABBOverlapCallback(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
    {
        URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).CString());
        URHO3D_PROFILE_FUNCTION();
        return 1;
    }


}
