#include "NewtonPhysicsWorld.h"
#include "IO/Log.h"
#include "NewtonCollisionShape.h"
#include "NewtonRigidBody.h"
#include "Newton.h"
#include "dMatrix.h"
#include "Core/Context.h"
#include "Core/CoreEvents.h"
#include "Core/Object.h"

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
            if(newtonWorld_ == nullptr)
                newtonWorld_ = NewtonCreate();
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

        if (newtonWorld_ != nullptr) {
            NewtonDestroy(newtonWorld_);
            newtonWorld_ = nullptr;
        }
    }



    void NewtonPhysicsWorld::HandleUpdate(StringHash eventType, VariantMap& eventData)
    {

        float timeStep = eventData[PreUpdate::P_TIMESTEP].GetFloat();
        NewtonUpdate(newtonWorld_, .0166666f);
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
