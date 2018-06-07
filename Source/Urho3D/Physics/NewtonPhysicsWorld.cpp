#include "NewtonPhysicsWorld.h"
#include "IO/Log.h"
#include "NewtonCollisionShape.h"
#include "NewtonRigidBody.h"


namespace Urho3D {
    NewtonPhysicsWorld::NewtonPhysicsWorld(Context* context) : Component(context)
    {

    }

    NewtonPhysicsWorld::~NewtonPhysicsWorld()
    {

    }

    void NewtonPhysicsWorld::RegisterObject(Context* context)
    {

    }

    void NewtonPhysicsWorld::Update(float timeStep)
    {
        URHO3D_LOGINFO("Updating Physics World..");
    }




    void RegisterPhysicsLibrary(Context* context)
    {
        NewtonCollisionShape::RegisterObject(context);
        NewtonRigidBody::RegisterObject(context);
        //Constraint::RegisterObject(context);
        NewtonPhysicsWorld::RegisterObject(context);
        //RaycastVehicle::RegisterObject(context);
    }
}
