#include "NewtonPhysicsWorld.h"
#include "IO/Log.h"
#include "NewtonCollisionShape.h"
#include "NewtonRigidBody.h"
#include "Newton.h"
#include "dMatrix.h"
#include "Core/Context.h"

namespace Urho3D {
    NewtonPhysicsWorld::NewtonPhysicsWorld(Context* context) : Component(context)
    {

    }

    NewtonPhysicsWorld::~NewtonPhysicsWorld()
    {
        freeWorld();
    }

    void NewtonPhysicsWorld::RegisterObject(Context* context)
    {
        context->RegisterFactory<NewtonPhysicsWorld>();
    }

    void NewtonPhysicsWorld::Update(float timeStep)
    {
        NewtonUpdateAsync(newtonWorld_, timeStep);
        NewtonWaitForUpdateToFinish(newtonWorld_);//hard syncronous wait for now.
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

    dMatrix UrhoToNewton(Matrix4 mat4)
    {
        return dMatrix(mat4.Data());
    }
    dMatrix UrhoToNewton(Matrix3x4 mat3x4)
    {
        return dMatrix(mat3x4.Data());
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
