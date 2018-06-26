#include "NewtonPhysicsWorld.h"
#include "IO/Log.h"
#include "NewtonCollisionShape.h"
#include "NewtonRigidBody.h"
#include "dMatrix.h"
#include "Core/Context.h"
#include "Core/CoreEvents.h"
#include "Core/Object.h"
#include "UrhoNewtonConversions.h"
#include "Scene/Component.h"
#include "Scene/Scene.h"
#include "Scene/Node.h"
#include "Math/Sphere.h"
#include "Container/Vector.h"

#include "Newton.h"

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




    void NewtonPhysicsWorld::SerializeNewtonWorld(String fileName)
    {
        NewtonSerializeToFile(newtonWorld_, fileName.CString(), nullptr, nullptr);
    }



    
    void NewtonPhysicsWorld::GetRigidBodies(PODVector<NewtonRigidBody*>& result, const Sphere& sphere, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {

        Matrix3x4 mat;
        mat.SetTranslation(sphere.center_);

        NewtonCollision* newtonShape = UrhoShapeToNewtonCollision(newtonWorld_, sphere, false);
        int numContacts = DoNewtonCollideTest(&UrhoToNewton(mat)[0][0], newtonShape);

        GetBodiesInConvexCast(result, numContacts);


    }

    void NewtonPhysicsWorld::GetRigidBodies(PODVector<NewtonRigidBody*>& result, const BoundingBox& box, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {
        Matrix3x4 mat;
        mat.SetTranslation(box.Center());

        NewtonCollision* newtonShape = UrhoShapeToNewtonCollision(newtonWorld_, box, false);
        int numContacts = DoNewtonCollideTest(&UrhoToNewton(mat)[0][0], newtonShape);

        GetBodiesInConvexCast(result, numContacts);


    }

    void NewtonPhysicsWorld::GetRigidBodies(PODVector<NewtonRigidBody*>& result, const NewtonRigidBody* body)
    {

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
            //#todo draw physics world specific things. joints?

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


    int NewtonPhysicsWorld::DoNewtonCollideTest(const float* const matrix, const NewtonCollision* shape)
    {

        return  NewtonWorldCollide(newtonWorld_,
            matrix, shape, nullptr,
            Newton_WorldRayPrefilterCallback, convexCastRetInfoArray,
            convexCastRetInfoSize_, 0);

    }

    void NewtonPhysicsWorld::GetBodiesInConvexCast(PODVector<NewtonRigidBody*>& result, int numContacts)
    {
        //iterate though contacts.
        for (int i = 0; i < numContacts; i++) {

            if (convexCastRetInfoArray[i].m_hitBody != nullptr) {

                void* userData = NewtonBodyGetUserData(convexCastRetInfoArray[i].m_hitBody);
                if (userData != nullptr)
                {
                    NewtonRigidBody* rigBody = static_cast<NewtonRigidBody*>(userData);
                    result.Push(rigBody);
                }
            }
        }
    }

    void Newton_ApplyForceAndTorqueCallback(const NewtonBody* body, dFloat timestep, int threadIndex)
    {
        static dFloat Ixx;
        static dFloat Iyy;
        static dFloat Izz;
        static dFloat mass;
        static dVector netForce;
        static dVector netTorque;


        //NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

        NewtonRigidBody* rigidBodyComp = static_cast<NewtonRigidBody*>(NewtonBodyGetUserData(body));

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

    unsigned Newton_WorldRayPrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
    {
        ///no filtering right now.
        return 1;///?
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
