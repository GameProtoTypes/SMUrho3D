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
#include "Scene/Scene.h"
#include "Scene/Node.h"

namespace Urho3D {


    static const Vector3 DEFAULT_GRAVITY = Vector3(0.0f, -9.81f, 0.0f);

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




    void NewtonPhysicsWorld::SetGravity(const Vector3& force)
    {
        gravity_ = force;
    }


    Urho3D::Vector3 NewtonPhysicsWorld::GetGravity()
{
        return gravity_;
    }

    void NewtonPhysicsWorld::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        if (debug)
        {
            //#todo draw physics world specific things.

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
                NewtonSetThreadsCount(newtonWorld_,4);
                //NewtonSetNumberOfSubsteps(newtonWorld_, 8);
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
        

        //apply the transform of all rigid body components to their respective nodes.
        for (NewtonRigidBody* rigBody : rigidBodyComponentList)
        {
            rigBody->ApplyTransform();
        }
    }


    void Newton_ApplyForceAndTorqueCallback(const NewtonBody* body, dFloat timestep, int threadIndex)
    {
        dFloat Ixx;
        dFloat Iyy;
        dFloat Izz;
        dFloat mass;


        // for this tutorial the only external force in the Gravity
        //NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

        NewtonRigidBody* rigidBodyComp = static_cast<NewtonRigidBody*>(NewtonBodyGetUserData(body));
        dVector netForce;
        dVector netTorque;
        rigidBodyComp->GetBakedForceAndTorque(netForce, netTorque);

        NewtonBodySetForce(body, &netForce[0]);
        NewtonBodySetTorque(body, &netTorque[0]);
        
    }


    void Newton_SetTransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
    {
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
