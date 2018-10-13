#include "PhysicsWorld.h"



namespace Urho3D
{

    float PhysicsWorld::SceneToPhysics_Domain(float x)
    {
        return x * physicsScale_;
    }

    //void PhysicsWorld::SceneToPhysics_Domain(float& x)
    //{
    //    x = x * physicsScale_;
    //}

    Urho3D::Vector3 PhysicsWorld::SceneToPhysics_Domain(Vector3 v)
    {
        return v * physicsScale_;
    }

    //void PhysicsWorld::SceneToPhysics_Domain(Vector3& v)
    //{
    //    v = v * physicsScale_;
    //}

    Urho3D::Matrix3x4 PhysicsWorld::SceneToPhysics_Domain(Matrix3x4 sceneFrame)
    {
        return GetPhysicsWorldFrame().Inverse() * sceneFrame;
    }

    //void PhysicsWorld::SceneToPhysics_Domain(Matrix3x4& frame)
    //{
    //    frame = GetPhysicsWorldFrame().Inverse() * frame;
    //}



    float PhysicsWorld::PhysicsToScene_Domain(float x)
    {
        return x / physicsScale_;
    }

    //void PhysicsWorld::PhysicsToScene_Domain(float& x)
    //{
    //    x = x / physicsScale_;
    //}

    Urho3D::Vector3 PhysicsWorld::PhysicsToScene_Domain(Vector3 v)
    {
        return v / physicsScale_;
    }

    //void PhysicsWorld::PhysicsToScene_Domain(Vector3& v)
    //{
    //    v = v / physicsScale_;
    //}

    Urho3D::Matrix3x4 PhysicsWorld::PhysicsToScene_Domain(Matrix3x4 newtonFrame)
    {
        return GetPhysicsWorldFrame() * newtonFrame;
    }

    //void PhysicsWorld::PhysicsToScene_Domain(Matrix3x4& frame)
    //{
    //    frame = GetPhysicsWorldFrame() * frame;
    //}

}
