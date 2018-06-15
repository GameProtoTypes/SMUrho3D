#include "NewtonPhysicsWorld.h"
#include "IO/Log.h"
#include "NewtonCollisionShape.h"
#include "NewtonRigidBody.h"
#include "Newton.h"
#include "dMatrix.h"
#include "Core/Context.h"
#include "Core/CoreEvents.h"
#include "Core/Object.h"
#include "UrhoNewtonConversions.h"
#include "Scene/Component.h"
#include "Scene/Node.h"

namespace Urho3D {
    NewtonPhysicsWorld::NewtonPhysicsWorld(Context* context) : Component(context)
    {
        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(NewtonPhysicsWorld, HandleUpdate));
    }

    NewtonPhysicsWorld::~NewtonPhysicsWorld()
    {
        freeWorld();
    }

    void NewtonPhysicsWorld::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonPhysicsWorld>();
    }




    void NewtonPhysicsWorld::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        if (debug)
        {
            //draw debug geometry on rigid bodies.
            for (NewtonRigidBody* body : rigidBodyComponentList) {
                body->DrawDebugGeometry(debug, depthTest);
            }
            
        }
    }

    void NewtonPhysicsWorld::OnSceneSet(Scene* scene)
    {
        if (scene) {
            //create the newton world
            if (newtonWorld_ == nullptr) {
                newtonWorld_ = NewtonCreate();
                NewtonSetSolverModel(newtonWorld_, 4);
                NewtonSetNumberOfSubsteps(newtonWorld_, 1);
            }
        }
        else
        {
            freeWorld();
        }
    }

    void NewtonPhysicsWorld::addCollisionShape(NewtonCollisionShape* collision)
    {
        collisionComponentList.Insert(0, WeakPtr<NewtonCollisionShape>(collision));
    }

    void NewtonPhysicsWorld::removeCollisionShape(NewtonCollisionShape* collision)
    {
        collisionComponentList.Remove(WeakPtr<NewtonCollisionShape>(collision));
    }

    void NewtonPhysicsWorld::addRigidBody(NewtonRigidBody* body)
    {
        rigidBodyComponentList.Insert(0, WeakPtr<NewtonRigidBody>(body));
    }

    void NewtonPhysicsWorld::removeRigidBody(NewtonRigidBody* body)
    {
        rigidBodyComponentList.Remove(WeakPtr<NewtonRigidBody>(body));
    }

    void NewtonPhysicsWorld::freeWorld()
    {
        //free any collision shapes currently in the list
        for (NewtonCollisionShape* col : collisionComponentList)
        {
            col->freeInternalCollision();
        }

        //free internal bodies for all rigid bodies.
        for (NewtonRigidBody* rgBody : rigidBodyComponentList)
        {
            rgBody->freeBody();
        }



        if (newtonWorld_ != nullptr) {
            NewtonDestroy(newtonWorld_);
            newtonWorld_ = nullptr;
        }
    }



    //gets called every 16 ms.
    void NewtonPhysicsWorld::HandleUpdate(StringHash eventType, VariantMap& eventData)
    {
        //use target time step to give newton constant time steps. timeStep = 0.01666666 in seconds.
        float timeStep = eventData[Update::P_TARGET_TIMESTEP].GetFloat();

        NewtonUpdate(newtonWorld_, timeStep);
        NewtonWaitForUpdateToFinish(newtonWorld_);
    }


    void Newton_ApplyForceAndTorqueCallback(const NewtonBody* body, dFloat timestep, int threadIndex)
    {
        dFloat Ixx;
        dFloat Iyy;
        dFloat Izz;
        dFloat mass;

        // for this tutorial the only external force in the Gravity
        NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

        dVector gravityForce(0.0f, mass * -9.8, 0.0f, 1.0f);

        NewtonBodySetForce(body, &gravityForce[0]);
    }


    void Newton_SetTransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
    {
        NewtonRigidBody* rigidBody = static_cast<NewtonRigidBody*>(NewtonBodyGetUserData(body));

        //Quaternion orientation;
        dMatrix newtMat(matrix);

        Vector3 translation;
        Quaternion orientation;
        Vector3 scale;
        NewtonToUrhoMat4(newtMat).Decompose(translation, orientation, scale);

        rigidBody->GetNode()->SetWorldTransform(translation, orientation);
    }


    void Newton_DestroyBodyCallback(const NewtonBody* body)
    {

    }























    void RegisterPhysicsLibrary(Context* context)
    {
        NewtonPhysicsWorld::RegisterObject(context);
        NewtonCollisionShape::RegisterObject(context);
        NewtonRigidBody::RegisterObject(context);
        //Constraint::RegisterObject(context);
        
        //RaycastVehicle::RegisterObject(context);
    }







}
