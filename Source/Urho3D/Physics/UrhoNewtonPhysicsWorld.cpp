#include "UrhoNewtonPhysicsWorld.h"
#include "IO/Log.h"
#include "NewtonCollisionShape.h"
#include "NewtonCollisionShapesDerived.h"
#include "NewtonRigidBody.h"
#include "dMatrix.h"
#include "Core/Context.h"
#include "Core/CoreEvents.h"
#include "Core/Object.h"
#include "UrhoNewtonConversions.h"
#include "Scene/Component.h"
#include "Scene/Scene.h"
#include "Scene/Node.h"
#include "../Graphics/Model.h"
#include "Math/Sphere.h"
#include "Container/Vector.h"

#include "Newton.h"
#include "NewtonMeshObject.h"
#include "Core/Thread.h"
#include "NewtonConstraint.h"
#include "NewtonFixedDistanceConstraint.h"
#include "Core/Profiler.h"
#include "PhysicsEvents.h"
#include "Graphics/VisualDebugger.h"
#include "NewtonPhysicsMaterial.h"
#include "NewtonBallAndSocketConstraint.h"
#include "NewtonKinematicsJoint.h"
#include "NewtonDebugDrawing.h"
#include "dgMatrix.h"

namespace Urho3D {


    static const Vector3 DEFAULT_GRAVITY = Vector3(0.0f, -9.81f, 0.0f);

    UrhoNewtonPhysicsWorld::UrhoNewtonPhysicsWorld(Context* context) : Component(context)
    {
        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(UrhoNewtonPhysicsWorld, HandleUpdate));

    }

    UrhoNewtonPhysicsWorld::~UrhoNewtonPhysicsWorld()
    {
        freeWorld();
    }

    void UrhoNewtonPhysicsWorld::RegisterObject(Context* context)
    {
        context->RegisterFactory<UrhoNewtonPhysicsWorld>(DEF_PHYSICS_CATEGORY.CString());
    }




    void UrhoNewtonPhysicsWorld::SerializeNewtonWorld(String fileName)
    {
        NewtonSerializeToFile(newtonWorld_, fileName.CString(), nullptr, nullptr);
    }



    
    String UrhoNewtonPhysicsWorld::GetSolverPluginName()
    {
        void* plugin = NewtonCurrentPlugin(newtonWorld_);
        return String(NewtonGetPluginString(newtonWorld_, plugin));
    }

    void UrhoNewtonPhysicsWorld::GetRigidBodies(PODVector<NewtonRigidBody*>& result, const Sphere& sphere, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {

        Matrix3x4 mat;
        mat.SetTranslation(sphere.center_);

        NewtonCollision* newtonShape = UrhoShapeToNewtonCollision(newtonWorld_, sphere, false);
        int numContacts = DoNewtonCollideTest(&UrhoToNewton(mat)[0][0], newtonShape);

        GetBodiesInConvexCast(result, numContacts);

        NewtonDestroyCollision(newtonShape);
    }

    void UrhoNewtonPhysicsWorld::GetRigidBodies(PODVector<NewtonRigidBody*>& result, const BoundingBox& box, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {
        Matrix3x4 mat;
        mat.SetTranslation(box.Center());

        NewtonCollision* newtonShape = UrhoShapeToNewtonCollision(newtonWorld_, box, false);
        int numContacts = DoNewtonCollideTest(&UrhoToNewton(mat)[0][0], newtonShape);

        GetBodiesInConvexCast(result, numContacts);
        NewtonDestroyCollision(newtonShape);

    }

    void UrhoNewtonPhysicsWorld::GetRigidBodies(PODVector<NewtonRigidBody*>& result, const NewtonRigidBody* body)
    {

    }



    void UrhoNewtonPhysicsWorld::SetGravity(const Vector3& force)
    {
        gravity_ = force;
    }


    Urho3D::Vector3 UrhoNewtonPhysicsWorld::GetGravity()
{
        return gravity_;
    }

    void UrhoNewtonPhysicsWorld::SetTotalIterations(int numIterations /*= 8*/)
    {
        totalIterationCount_ = numIterations;
        applyNewtonWorldSettings();
    }


    void UrhoNewtonPhysicsWorld::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        if (debug)
        {
            //draw scene collision
            if(node_->HasComponent<NewtonRigidBody>())
                GetComponent<NewtonRigidBody>()->DrawDebugGeometry(debug, depthTest);
            //NewtonCollisionDraw(sceneCollision_, Matrix3x4(), debug, depthTest);


            //#todo draw physics world specific things. joints?
            for (NewtonConstraint* consttraint : constraintList)
            {
                consttraint->DrawDebugGeometry(debug, depthTest);
            }

            //draw debug geometry on rigid bodies.
            for (NewtonRigidBody* body : rigidBodyComponentList) {
                body->DrawDebugGeometry(debug, depthTest);
            }

            //draw debug geomtry on collision shapes that are not owned by a rigid body
            PODVector<NewtonCollisionShape*> aloneCollisionShapes;
            GetAloneCollisionShapes(aloneCollisionShapes, node_, true);

            for (NewtonCollisionShape* col : aloneCollisionShapes)
            {
                col->DrawDebugGeometry(debug, depthTest);
            }

        }
    }

    void UrhoNewtonPhysicsWorld::OnSceneSet(Scene* scene)
    {
        if (scene) {
            //create the newton world
            if (newtonWorld_ == nullptr) {
                newtonWorld_ = NewtonCreate();
                NewtonWorldSetUserData(newtonWorld_, (void*)this);
                applyNewtonWorldSettings();



                //create the scene collision
                sceneCollision_ = NewtonCreateSceneCollision(newtonWorld_, 0);

                reBuildSceneRigidBody();

            }
        }
        else
        {
            freeWorld();
        }
    }

    void UrhoNewtonPhysicsWorld::addCollisionShape(NewtonCollisionShape* collision)
    {
        collisionComponentList.Insert(0, WeakPtr<NewtonCollisionShape>(collision));
    }

    void UrhoNewtonPhysicsWorld::removeCollisionShape(NewtonCollisionShape* collision)
    {
        collisionComponentList.Remove(WeakPtr<NewtonCollisionShape>(collision));
    }

    void UrhoNewtonPhysicsWorld::addRigidBody(NewtonRigidBody* body)
    {
        rigidBodyComponentList.Insert(0, WeakPtr<NewtonRigidBody>(body));
    }

    void UrhoNewtonPhysicsWorld::removeRigidBody(NewtonRigidBody* body)
    {
        rigidBodyComponentList.Remove(WeakPtr<NewtonRigidBody>(body));
    }

    void UrhoNewtonPhysicsWorld::addConstraint(NewtonConstraint* constraint)
    {
        constraintList.Insert(0, WeakPtr<NewtonConstraint>(constraint));
    }

    void UrhoNewtonPhysicsWorld::removeConstraint(NewtonConstraint* constraint)
    {
        constraintList.Remove(WeakPtr<NewtonConstraint>(constraint));
    }

    void UrhoNewtonPhysicsWorld::addPhysicsMaterial(NewtonPhysicsMaterial* material)
    {
        if (physMaterialList.Contains(SharedPtr<NewtonPhysicsMaterial>(material)))
            return;
        else
        {
            physMaterialList.Insert(0, SharedPtr<NewtonPhysicsMaterial>(material));
            material->newtonGroupId = NewtonMaterialCreateGroupID(newtonWorld_);



            computeMaterialPairs();
        }
    }

    void UrhoNewtonPhysicsWorld::computeMaterialPairs()
    {
        physMaterialPairList.Clear();
        for (NewtonPhysicsMaterial* mat1 : physMaterialList) {
            for (NewtonPhysicsMaterial* mat2 : physMaterialList) {
                SharedPtr<NewtonPhysicsMaterialContactPair> newPair = context_->CreateObject<NewtonPhysicsMaterialContactPair>();
                newPair->SetMaterials(mat1, mat2);//compute.
                physMaterialPairList.Insert(0, newPair);
            }
        }
    }

    void UrhoNewtonPhysicsWorld::freeWorld()
    {

        //free any joints
        for (NewtonConstraint* constraint : constraintList)
        {
            constraint->freeConstraint();
        }
        constraintList.Clear();

        //free any collision shapes currently in the list
        for (NewtonCollisionShape* col : collisionComponentList)
        {
            col->freeInternalCollision();
        }
        collisionComponentList.Clear();


        //free scene collision
        if (sceneCollision_) {
            NewtonDestroyCollision(sceneCollision_);
            sceneCollision_ = nullptr;
        }

        //free internal bodies for all rigid bodies.
        for (NewtonRigidBody* rgBody : rigidBodyComponentList)
        {
            rgBody->freeBody();
        }
        rigidBodyComponentList.Clear();

        //free meshes in mesh cache
        newtonMeshCache_.Clear();


       

        //destroy newton world.
        if (newtonWorld_ != nullptr) {
            NewtonDestroy(newtonWorld_);
            newtonWorld_ = nullptr;
        }
    }




    void UrhoNewtonPhysicsWorld::applyNewtonWorldSettings()
    {
        NewtonSetSolverModel(newtonWorld_, totalIterationCount_);
        NewtonSetThreadsCount(newtonWorld_, newtonThreadCount_);
        NewtonSelectBroadphaseAlgorithm(newtonWorld_, 1);//persistent broadphase.
    }

    void UrhoNewtonPhysicsWorld::HandleUpdate(StringHash eventType, VariantMap& eventData)
    {
        URHO3D_PROFILE_FUNCTION();

           
        //rebuild collision shapes from child nodes to root nodes.
        rebuildDirtyPhysicsComponents();
        {
            URHO3D_PROFILE("NewtonUpdate");
            //use target time step to give newton constant time steps. 
            float timeStep = eventData[Update::P_TARGET_TIMESTEP].GetFloat();

            //send prestep physics event
            VariantMap eventData;
            eventData[PhysicsPreStep::P_WORLD] = this;
            eventData[PhysicsPreStep::P_TIMESTEP] = timeStep;
            SendEvent(E_PHYSICSPRESTEP, eventData);


            NewtonUpdate(newtonWorld_, timeStep);
            NewtonWaitForUpdateToFinish(newtonWorld_);

            //send post physics event.
            SendEvent(E_PHYSICSPOSTSTEP, eventData);
        }

        {
            URHO3D_PROFILE("Apply Node Transforms");
            //apply the transform of all rigid body components to their respective nodes.
            for (NewtonRigidBody* rigBody : rigidBodyComponentList)
            {
                if (rigBody->GetInternalTransformDirty()) {
                    rigBody->ApplyTransform();
                    rigBody->MarkInternalTransformDirty(false);
                }
            }
        }

    }



    void UrhoNewtonPhysicsWorld::rebuildDirtyPhysicsComponents()
    {
        URHO3D_PROFILE_FUNCTION();



        //rebuild dirty collision shapes
        for (NewtonCollisionShape* colShape : collisionComponentList)
        {
            if (colShape->GetDirty()) {
                colShape->updateBuild();
                GSS<VisualDebugger>()->AddOrb(colShape->GetNode()->GetWorldPosition(), 1.0f, Color::GREEN);
                colShape->MarkDirty(false);
            }
        }
        

        //then rebuild rigid bodies if they need rebuilt (dirty) from root nodes up.
        for (NewtonRigidBody* rigBody : rigidBodyComponentList)
        {
            if (!rigBody->GetDirty())
                continue;

            NewtonRigidBody* mostRootRigBody = GetMostRootRigidBody(rigBody->GetNode());

            if (mostRootRigBody->needsRebuilt_){
                mostRootRigBody->reBuildBody();
                GSS<VisualDebugger>()->AddOrb(mostRootRigBody->GetNode()->GetWorldPosition(), 1.0f, Color::BLUE);
                mostRootRigBody->MarkDirty(false);
            }
        }

        for (NewtonRigidBody* rigBody : rigidBodyComponentList)
        {
            rigBody->applyDefferedActions();
        }



        //rebuild contraints if they need rebuilt (dirty)
        for (NewtonConstraint* constraint : constraintList)
        {
            if (constraint->needsRebuilt_)
                constraint->reEvalConstraint();
        }
    }

    void UrhoNewtonPhysicsWorld::reBuildSceneRigidBody()
    {
        if (sceneRigidBody_)
        {
            NewtonDestroyBody(sceneRigidBody_);
            sceneRigidBody_ = nullptr;
        }

        dMatrix mat = dGetIdentityMatrix() ;
        sceneRigidBody_ = NewtonCreateDynamicBody(newtonWorld_, sceneCollision_, &mat[0][0]);


    }

    int UrhoNewtonPhysicsWorld::DoNewtonCollideTest(const float* const matrix, const NewtonCollision* shape)
    {

        return  NewtonWorldCollide(newtonWorld_,
            matrix, shape, nullptr,
            Newton_WorldRayPrefilterCallback, convexCastRetInfoArray,
            convexCastRetInfoSize_, 0);

    }

    void UrhoNewtonPhysicsWorld::GetBodiesInConvexCast(PODVector<NewtonRigidBody*>& result, int numContacts)
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

    Urho3D::StringHash UrhoNewtonPhysicsWorld::NewtonMeshKey(String modelResourceName, int modelLodLevel, String otherData)
    {
        return modelResourceName + String(modelLodLevel) + otherData;
    }

    NewtonMeshObject* UrhoNewtonPhysicsWorld::GetCreateNewtonMesh(StringHash urhoNewtonMeshKey)
    {
        if (newtonMeshCache_.Contains(urhoNewtonMeshKey)) {
            return newtonMeshCache_[urhoNewtonMeshKey];
        }
        else
        {
            NewtonMesh* mesh = NewtonMeshCreate(newtonWorld_);
            SharedPtr<NewtonMeshObject> meshObj = context_->CreateObject<NewtonMeshObject>();
            meshObj->mesh = mesh;
            newtonMeshCache_[urhoNewtonMeshKey] = meshObj;
            return meshObj;
        }
    }

    NewtonMeshObject* UrhoNewtonPhysicsWorld::GetNewtonMesh(StringHash urhoNewtonMeshKey)
    {
        if (newtonMeshCache_.Contains(urhoNewtonMeshKey)) {
            return newtonMeshCache_[urhoNewtonMeshKey];
        }
        return nullptr;
    }

 

    String NewtonThreadProfilerString(int threadIndex)
    {
        return (String("Newton_Thread") + String(threadIndex));
    }

    void Newton_ApplyForceAndTorqueCallback(const NewtonBody* body, dFloat timestep, int threadIndex)
    {
        URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).CString());
        URHO3D_PROFILE_FUNCTION()


        dVector netForce;
        dVector netTorque;
        NewtonRigidBody* rigidBodyComp = nullptr;

        rigidBodyComp = static_cast<NewtonRigidBody*>(NewtonBodyGetUserData(body));


        rigidBodyComp->GetForceAndTorque(netForce, netTorque);
        

        NewtonBodySetForce(body, &netForce[0]);
        NewtonBodySetTorque(body, &netTorque[0]);
    }


    void Newton_SetTransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
    {
        NewtonRigidBody* rigBody = static_cast<NewtonRigidBody*>(NewtonBodyGetUserData(body));
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
        URHO3D_PROFILE_FUNCTION();;


        //#todo
        //const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
        //const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);



        //for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
        //    NewtonMaterial* const material = NewtonContactGetMaterial(contact);
        //    float frictionValue = 0.2;

        //    //NewtonMaterialGet


        //    NewtonMaterialSetContactFrictionCoef(material, frictionValue + 0.1f, frictionValue, 0);
        //    NewtonMaterialSetContactFrictionCoef(material, frictionValue + 0.1f, frictionValue, 1);
        //}
    }

    int Newton_AABBOverlapCallback(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
    {
        URHO3D_PROFILE_THREAD(NewtonThreadProfilerString(threadIndex).CString());
        URHO3D_PROFILE_FUNCTION();
        return 1;
    }

    //traverses towards the scene root returning the first found rigid body.
    NewtonRigidBody* GetMostRootRigidBody(Node* node)
    {
        NewtonRigidBody* mostRootRigidBody = node->GetComponent<NewtonRigidBody>();
        if (!mostRootRigidBody)
            mostRootRigidBody = node->GetParentComponent<NewtonRigidBody>(true);

        while (mostRootRigidBody)//keep going up.
        {
            NewtonRigidBody* rig = mostRootRigidBody->GetNode()->GetParentComponent<NewtonRigidBody>(true);
            if (rig)
                mostRootRigidBody = rig;
            else
                break;
        }


        return mostRootRigidBody;
    }



    //recurses up the scene tree starting a node and continuing up every branch adding collision shapes to the array until a rigid body is encountered in which case the algorithm stops traversing that branch.
    void GetAloneCollisionShapes(PODVector<NewtonCollisionShape*>& colShapes, Node* startingNode_, bool includeStartingNode, bool recurse)
    {
        PODVector<NewtonCollisionShape*> colList;
        if (includeStartingNode)
        {

            if (!startingNode_->HasComponent<NewtonRigidBody>() && startingNode_->GetDerivedComponent<NewtonCollisionShape>()) {

                PODVector<NewtonCollisionShape*> compsOnNode;
                startingNode_->GetDerivedComponents(compsOnNode, false, true);
                colList += compsOnNode;
            }
        }

        PODVector<Node*> childrenWithCollisionShapes;
        startingNode_->GetChildrenWithDerivedComponent<NewtonCollisionShape>(childrenWithCollisionShapes, false);


        //trim out the children with rigid bodies from the list
        for (Node* node : childrenWithCollisionShapes)
        {
            if (!node->HasComponent<NewtonRigidBody>())
            {
                PODVector<NewtonCollisionShape*> compsOnNode;
                startingNode_->GetDerivedComponents(compsOnNode, false, true);
                colList += compsOnNode;
            }          
        }

        //add to final list
        colShapes += colList;

        if (recurse) {
            for (NewtonCollisionShape* col : colList)
            {
                GetAloneCollisionShapes(colShapes, col->GetNode(), false);
            }
        }
    }









    void RegisterPhysicsLibrary(Context* context)
    {
        UrhoNewtonPhysicsWorld::RegisterObject(context);

        NewtonCollisionShape::RegisterObject(context);
        NewtonCollisionShape_Box::RegisterObject(context);
        NewtonCollisionShape_Sphere::RegisterObject(context);
        NewtonCollisionShape_Cylinder::RegisterObject(context);
        NewtonCollisionShape_Capsule::RegisterObject(context);
        NewtonCollisionShape_Cone::RegisterObject(context);
        NewtonCollisionShape_Geometry::RegisterObject(context);
        NewtonCollisionShape_ConvexHull::RegisterObject(context);
        NewtonCollisionShape_ConvexHullCompound::RegisterObject(context);
        NewtonCollisionShape_ConvexDecompositionCompound::RegisterObject(context);
        NewtonCollisionShape_SceneCollision::RegisterObject(context);

        NewtonRigidBody::RegisterObject(context);
        NewtonMeshObject::RegisterObject(context);
        NewtonConstraint::RegisterObject(context);
        NewtonFixedDistanceConstraint::RegisterObject(context);
        NewtonBallAndSocketConstraint::RegisterObject(context);
        NewtonKinematicsConstraint::RegisterObject(context);
        //Constraint::RegisterObject(context);
        
        //RaycastVehicle::RegisterObject(context);

        NewtonPhysicsMaterial::RegisterObject(context);
        NewtonPhysicsMaterialContactPair::RegisterObject(context);
    }








}
