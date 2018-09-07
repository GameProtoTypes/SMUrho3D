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

        

        unsigned int key = IntVector2(rigBody0->GetID(), rigBody1->GetID()).ToHash();

        PhysicsWorld* physicsWorld = rigBody0->GetPhysicsWorld();

        if (physicsWorld == nullptr)
            return;//scene is being destroyed.

        SharedPtr<RigidBodyContactEntry> contactEntry = nullptr;

        //get/create the struct into the physicsWorld body contact state map
        NewtonWorldCriticalSectionLock(physicsWorld->GetNewtonWorld(), threadIndex);
            contactEntry = physicsWorld->GetCreateBodyContactEntry(key);
        NewtonWorldCriticalSectionUnlock(physicsWorld->GetNewtonWorld());


        contactEntry->body0 = rigBody0;
        contactEntry->body1 = rigBody1;
        contactEntry->inContactProgress = true;
        contactEntry->numContacts = NewtonContactJointGetContactCount(contactJoint);
        contactEntry->contactPositions.Resize(contactEntry->numContacts);
        contactEntry->contactNormals.Resize(contactEntry->numContacts);

        int contactIdx = 0;
        for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {


            NewtonMaterial* const material = NewtonContactGetMaterial(contact);

            //extract the urho material pair Data from the newton material pair.
            PhysicsMaterialContactPair* pairData = static_cast<PhysicsMaterialContactPair*>(NewtonMaterialGetMaterialPairUserData(material));

            //apply material settings to contact.
            NewtonMaterialSetContactFrictionCoef(material, pairData->staticFrictionCoef_, pairData->kineticFrictionCoef_, 0);
            NewtonMaterialSetContactElasticity(material, pairData->elasticity_);
            NewtonMaterialSetContactSoftness(material, pairData->softness_);

            //get contact geometric info for the contact struct

            dVector pos, norm;
            NewtonMaterialGetContactPositionAndNormal(material, body0, &pos[0], &norm[0]);
            contactEntry->contactNormals[contactIdx] = NewtonToUrhoVec3(norm);
            contactEntry->contactPositions[contactIdx] = NewtonToUrhoVec3(pos);


            contactIdx++;
        }






    }






    int Newton_AABBOverlapCallback(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
    {
        URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).CString());
        URHO3D_PROFILE_FUNCTION();
        return 1;
    }


}
