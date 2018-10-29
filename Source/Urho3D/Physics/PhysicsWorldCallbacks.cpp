#include "PhysicsWorld.h"
#include "Core/Profiler.h"
#include "RigidBody.h"
#include "Scene/Component.h"
#include "Scene/Scene.h"
#include "UrhoNewtonConversions.h"
#include "IO/Log.h"
#include "Newton.h"
#include "Graphics/VisualDebugger.h"


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
        {
            PhysicsWorld* physicsWorld = rigidBodyComp->GetScene()->GetComponent<PhysicsWorld>();
            float physicsScale = physicsWorld->GetPhysicsScale();
            gravityForce = physicsWorld->GetGravity() * physicsScale* rigidBodyComp->GetEffectiveMass();


            //all 
            netForce *= physicsScale*physicsScale*physicsScale;


            netForce += gravityForce;




            NewtonBodySetForce(body, &UrhoToNewton(netForce)[0]);
            NewtonBodySetTorque(body, &UrhoToNewton(netTorque*physicsScale*physicsScale*physicsScale*physicsScale*physicsScale)[0]);

        }
    }


    void Newton_SetTransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
    {
        RigidBody* rigBody = static_cast<RigidBody*>(NewtonBodyGetUserData(body));
        if(rigBody)
            rigBody->MarkInternalTransformDirty();
    }


    void Newton_DestroyBodyCallback(const NewtonBody* body)
    {

    }



    dFloat Newton_WorldRayCastFilterCallback(const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam)
    {
        PhysicsRayCastUserData*  data = (PhysicsRayCastUserData*)userData;

        PhysicsRayCastIntersection intersection;
        intersection.body = (NewtonBody*)body;
        intersection.rayIntersectParameter = intersetParam;
        intersection.rayIntersectWorldPosition = NewtonToUrhoVec3(dVector(contact));
        intersection.rayIntersectWorldNormal = NewtonToUrhoVec3(dVector(normal));
        intersection.rigBody = (RigidBody*)NewtonBodyGetUserData(body);
        data->intersections += intersection;


        //continue
        return 1.0f;
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



        for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {




            NewtonMaterial* const material = NewtonContactGetMaterial(contact);

            NewtonCollision* shape0 = NewtonMaterialGetBodyCollidingShape(material, body0);
            NewtonCollision* shape1 = NewtonMaterialGetBodyCollidingShape(material, body1);


            CollisionShape* colShape0 = static_cast<CollisionShape*>(NewtonCollisionGetUserData(shape0));
            CollisionShape* colShape1 = static_cast<CollisionShape*>(NewtonCollisionGetUserData(shape1));



            float staticFriction0 = colShape0->GetStaticFriction();
            float kineticFriction0 = colShape0->GetKineticFriction();
            float elasticity0 = colShape0->GetElasticity();
            float softness0 = colShape0->GetSoftness();

            float staticFriction1 = colShape1->GetStaticFriction();
            float kineticFriction1 = colShape1->GetKineticFriction();
            float elasticity1 = colShape1->GetElasticity();
            float softness1 = colShape1->GetSoftness();


            float finalStaticFriction = Max(staticFriction0, staticFriction1);
            float finalKineticFriction = Max(kineticFriction0, kineticFriction1);
            float finalElasticity = Min(elasticity0, elasticity1);
            float finalSoftness = Max(softness0, softness1);

            //apply material settings to contact.
            NewtonMaterialSetContactFrictionCoef(material, finalStaticFriction, finalKineticFriction, 0);
            NewtonMaterialSetContactElasticity(material, finalElasticity);
            NewtonMaterialSetContactSoftness(material, finalSoftness);

            if (rigBody0->GetTriggerMode() || rigBody1->GetTriggerMode()) {
                NewtonContactJointRemoveContact(contactJoint, contact);
                continue;
            }
        }






    }






    int Newton_AABBOverlapCallback(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
    {
        URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).CString());
        URHO3D_PROFILE_FUNCTION();


        const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
        const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);

        RigidBody* rigBody0 = static_cast<RigidBody*>(NewtonBodyGetUserData(body0));
        RigidBody* rigBody1 = static_cast<RigidBody*>(NewtonBodyGetUserData(body1));


        PhysicsWorld* physicsWorld = rigBody0->GetPhysicsWorld();

        if (physicsWorld == nullptr)
            return 1;//scene is being destroyed.

        //#todo why is this happening.
        if (!rigBody0->RefCountPtr() || !rigBody1->RefCountPtr())
        {
            return 1;
        }


        bool res;
        NewtonWorldCriticalSectionLock(physicsWorld->GetNewtonWorld(), threadIndex);

        res = rigBody1->CanCollideWith(rigBody0);


        NewtonWorldCriticalSectionUnlock(physicsWorld->GetNewtonWorld());
        return res;
    }


    int Newton_AABBCompoundOverlapCallback(const NewtonJoint* const contact, dFloat timestep, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex)
    {
        URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).CString());
        URHO3D_PROFILE_FUNCTION();

        return 1;
    }

}
