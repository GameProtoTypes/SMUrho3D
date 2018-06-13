#include "NewtonPhysicsWorld.h"
#include "IO/Log.h"
#include "NewtonCollisionShape.h"
#include "NewtonRigidBody.h"
#include "Newton.h"
#include "dMatrix.h"

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
        NewtonUpdateAsync(newtonWorld_, timeStep);
        NewtonWaitForUpdateToFinish(newtonWorld_);//hard syncronous wait for now.
    }




    void NewtonPhysicsWorld::OnSceneSet(Scene* scene)
    {
        //create the newton world
        newtonWorld_ = NewtonCreate();
    }

    dMatrix UrhoToNewton(Matrix4 mat)
    {
        return dMatrix(mat.Data());
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
