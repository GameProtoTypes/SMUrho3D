#include "UrhoNewtonPhysicsWorld.h"
#include "IO/Log.h"
#include "NewtonCollisionShape.h"
#include "NewtonCollisionShapesDerived.h"
#include "RigidBody.h"
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
#include "dCustomJoint.h"

namespace Urho3D {


    static const Vector3 DEFAULT_GRAVITY = Vector3(0.0f, -9.81f, 0.0f);

    UrhoNewtonPhysicsWorld::UrhoNewtonPhysicsWorld(Context* context) : Component(context)
    {
        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(UrhoNewtonPhysicsWorld, HandleUpdate));

    }

    UrhoNewtonPhysicsWorld::~UrhoNewtonPhysicsWorld()
    {
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

    void UrhoNewtonPhysicsWorld::GetRigidBodies(PODVector<RigidBody*>& result, const Sphere& sphere, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {

        Matrix3x4 mat;
        mat.SetTranslation(sphere.center_);

        NewtonCollision* newtonShape = UrhoShapeToNewtonCollision(newtonWorld_, sphere, false);
        int numContacts = DoNewtonCollideTest(&UrhoToNewton(mat)[0][0], newtonShape);

        GetBodiesInConvexCast(result, numContacts);

        NewtonDestroyCollision(newtonShape);
    }

    void UrhoNewtonPhysicsWorld::GetRigidBodies(PODVector<RigidBody*>& result, const BoundingBox& box, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {
        Matrix3x4 mat;
        mat.SetTranslation(box.Center());

        NewtonCollision* newtonShape = UrhoShapeToNewtonCollision(newtonWorld_, box, false);
        int numContacts = DoNewtonCollideTest(&UrhoToNewton(mat)[0][0], newtonShape);

        GetBodiesInConvexCast(result, numContacts);
        NewtonDestroyCollision(newtonShape);

    }


    void UrhoNewtonPhysicsWorld::GetRigidBodies(PODVector<RigidBody*>& result, const RigidBody* body)
    {
        dMatrix mat;
        NewtonBodyGetMatrix(body->GetNewtonBody(), &mat[0][0]);
        NewtonCollision* newtonShape = body->GetEffectiveNewtonCollision();
        int numContacts = DoNewtonCollideTest(&mat[0][0], newtonShape);

        GetBodiesInConvexCast(result, numContacts);
    }



    void UrhoNewtonPhysicsWorld::SetGravity(const Vector3& force)
    {
        gravity_ = force;
    }


    Urho3D::Vector3 UrhoNewtonPhysicsWorld::GetGravity()
{
        return gravity_;
    }

    void UrhoNewtonPhysicsWorld::SetIterationCount(int numIterations /*= 8*/)
    {
        iterationCount_ = numIterations;
        applyNewtonWorldSettings();
    }


    int UrhoNewtonPhysicsWorld::GetIterationCount() const
    {
        return iterationCount_;
    }

    void UrhoNewtonPhysicsWorld::SetSubstepCount(int numSubsteps)
    {
        numSubsteps_ = numSubsteps;
        applyNewtonWorldSettings();
    }

    int UrhoNewtonPhysicsWorld::GetSubstepCount() const
    {
        return numSubsteps_;
    }

    void UrhoNewtonPhysicsWorld::SetThreadCount(int numThreads)
    {
        newtonThreadCount_ = numThreads;
        applyNewtonWorldSettings();
    }

    int UrhoNewtonPhysicsWorld::GetThreadCount() const
    {
        return newtonThreadCount_;
    }

    void UrhoNewtonPhysicsWorld::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        URHO3D_PROFILE_FUNCTION();
        if (debug)
        {
            //draw debug geometry on constraints.
            for (NewtonConstraint* consttraint : constraintList)
            {
                consttraint->DrawDebugGeometry(debug, depthTest);
            }

            //draw debug geometry on rigid bodies.
            for (RigidBody* body : rigidBodyComponentList) {
                body->DrawDebugGeometry(debug, depthTest);
            }

            //draw debug geometry on static scene.
            if (sceneBody_)
                sceneBody_->DrawDebugGeometry(debug, depthTest);


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

                sceneBody_ = scene->CreateComponent<RigidBody>();
                sceneBody_->SetIsSceneRootBody(true);
            }
        }
        else
        {
            //wait for update to finish if in async mode so we can safely clean up.
            NewtonWaitForUpdateToFinish(newtonWorld_);

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

    void UrhoNewtonPhysicsWorld::addRigidBody(RigidBody* body)
    {
        rigidBodyComponentList.Insert(0, WeakPtr<RigidBody>(body));
    }

    void UrhoNewtonPhysicsWorld::removeRigidBody(RigidBody* body)
    {
        rigidBodyComponentList.Remove(WeakPtr<RigidBody>(body));
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
            constraint->freeInternal();
        }
        constraintList.Clear();

        //free any collision shapes currently in the list
        for (NewtonCollisionShape* col : collisionComponentList)
        {
            col->freeInternalCollision();
        }
        collisionComponentList.Clear();



        //free internal bodies for all rigid bodies.
        for (RigidBody* rgBody : rigidBodyComponentList)
        {
            rgBody->freeBody();
        }
        rigidBodyComponentList.Clear();

        //free meshes in mesh cache
        newtonMeshCache_.Clear();


        freePhysicsInternals();


        //destroy newton world.
        if (newtonWorld_ != nullptr) {
            NewtonDestroy(newtonWorld_);
            newtonWorld_ = nullptr;
        }
    }




    void UrhoNewtonPhysicsWorld::addToFreeQueue(NewtonBody* newtonBody)
    {
        freeBodyQueue_ += newtonBody;
    }

    void UrhoNewtonPhysicsWorld::addToFreeQueue(dCustomJoint* newtonConstraint)
    {
        freeConstraintQueue_ += newtonConstraint;
    }

    void UrhoNewtonPhysicsWorld::addToFreeQueue(NewtonCollision* newtonCollision)
    {
        freeCollisionQueue_ += newtonCollision;
    }

    void UrhoNewtonPhysicsWorld::applyNewtonWorldSettings()
    {
        NewtonSetSolverModel(newtonWorld_, iterationCount_);
        NewtonSetNumberOfSubsteps(newtonWorld_, numSubsteps_);
        NewtonSetThreadsCount(newtonWorld_, newtonThreadCount_);
        NewtonSelectBroadphaseAlgorithm(newtonWorld_, 1);//persistent broadphase.
    }

    void UrhoNewtonPhysicsWorld::HandleUpdate(StringHash eventType, VariantMap& eventData)
    {
        URHO3D_PROFILE_FUNCTION();
        float timeStep = eventData[Update::P_TARGET_TIMESTEP].GetFloat();

        //send prestep physics event
        VariantMap sendEventData;
        sendEventData[PhysicsPreStep::P_WORLD] = this;
        sendEventData[PhysicsPreStep::P_TIMESTEP] = timeStep;
        {
            URHO3D_PROFILE("Wait For ASync Update To finish.");
            NewtonWaitForUpdateToFinish(newtonWorld_);
        }

        //send post physics event.
        SendEvent(E_PHYSICSPOSTSTEP, sendEventData);


        {
            URHO3D_PROFILE("Apply Node Transforms");
            //apply the transform of all rigid body components to their respective nodes.
            for (RigidBody* rigBody : rigidBodyComponentList)
            {
                if (rigBody->GetInternalTransformDirty()) {
                    rigBody->ApplyTransform();
                    rigBody->MarkInternalTransformDirty(false);
                }
            }
        }


        //rebuild collision shapes from child nodes to root nodes.
        rebuildDirtyPhysicsComponents();
        freePhysicsInternals();

        {
            URHO3D_PROFILE("NewtonUpdate");
            //use target time step to give newton constant time steps. 

            SendEvent(E_PHYSICSPRESTEP, sendEventData);
            NewtonUpdateAsync(newtonWorld_, timeStep);
           // NewtonWaitForUpdateToFinish(newtonWorld_);

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
                colShape->MarkDirty(false);
            }
        }
        

        //then rebuild rigid bodies if they need rebuilt (dirty) from root nodes up.
        for (RigidBody* rigBody : rigidBodyComponentList)
        {
            if (!rigBody->GetDirty())
                continue;

            if (rigBody == sceneBody_)//special case scene
            {
                rigBody->reBuildBody();
                rigBody->MarkDirty(false);
                continue;
            }


            //trigger a rebuild on the root of the new tree.
            PODVector<RigidBody*> rigBodies;
            GetRootRigidBodies(rigBodies, rigBody->GetNode(), false);
            RigidBody* mostRootRigBody = rigBodies.Back();
            if (mostRootRigBody->needsRebuilt_){
                mostRootRigBody->reBuildBody();
                mostRootRigBody->MarkDirty(false);
            }
        }

        for (RigidBody* rigBody : rigidBodyComponentList)
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


    int UrhoNewtonPhysicsWorld::DoNewtonCollideTest(const float* const matrix, const NewtonCollision* shape)
    {

        return  NewtonWorldCollide(newtonWorld_,
            matrix, shape, nullptr,
            Newton_WorldRayPrefilterCallback, convexCastRetInfoArray,
            convexCastRetInfoSize_, 0);

    }

    void UrhoNewtonPhysicsWorld::GetBodiesInConvexCast(PODVector<RigidBody*>& result, int numContacts)
    {
        //iterate though contacts.
        for (int i = 0; i < numContacts; i++) {

            if (convexCastRetInfoArray[i].m_hitBody != nullptr) {

                void* userData = NewtonBodyGetUserData(convexCastRetInfoArray[i].m_hitBody);
                if (userData != nullptr)
                {
                    RigidBody* rigBody = static_cast<RigidBody*>(userData);
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

    void UrhoNewtonPhysicsWorld::freePhysicsInternals()
    {
        for (dCustomJoint* constraint : freeConstraintQueue_)
        {
            delete constraint;
        }
        freeConstraintQueue_.Clear();


        for (NewtonCollision* col : freeCollisionQueue_)
        {
            NewtonDestroyCollision(col);
        }
        freeCollisionQueue_.Clear();


        for (NewtonBody* body : freeBodyQueue_)
        {
            NewtonDestroyBody(body);
        }
        freeBodyQueue_.Clear();




    }
 

    String NewtonThreadProfilerString(int threadIndex)
    {
        return (String("Newton_Thread") + String(threadIndex));
    }

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
        if(rigidBodyComp->GetScene())//on scene destruction sometimes this is null so check...
            gravityForce = rigidBodyComp->GetScene()->GetComponent<UrhoNewtonPhysicsWorld>()->GetGravity() * rigidBodyComp->GetEffectiveMass();

        netForce += gravityForce;

        NewtonBodySetForce(body, &UrhoToNewton(netForce)[0]);
        NewtonBodySetTorque(body, &UrhoToNewton(netTorque)[0]);
    }


    void Newton_SetTransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
    {
        RigidBody* rigBody = static_cast<RigidBody*>(NewtonBodyGetUserData(body));
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


    //add rigid bodies to the list as the function recurses from node to root. the last rigidbody in rigidBodies is the most root. optionally include the scene as root.
    void GetRootRigidBodies(PODVector<RigidBody*>& rigidBodies, Node* node, bool includeScene)
    {
        RigidBody* body = node->GetComponent<RigidBody>();


        if (body)
            rigidBodies += body;

        //recurse on parent
        if(node->GetParent() && ((node->GetScene() != node->GetParent()) || includeScene))
            GetRootRigidBodies(rigidBodies, node->GetParent(), includeScene);
    }



    //recurses up the scene tree starting a node and continuing up every branch adding collision shapes to the array until a rigid body is encountered in which case the algorithm stops traversing that branch.
    void GetAloneCollisionShapes(PODVector<NewtonCollisionShape*>& colShapes, Node* startingNode_, bool includeStartingNode, bool recurse)
    {
        PODVector<NewtonCollisionShape*> colList;
        if (includeStartingNode)
        {
            if (startingNode_->HasComponent<RigidBody>())
                return;

            if ((!startingNode_->HasComponent<RigidBody>()) && startingNode_->GetDerivedComponent<NewtonCollisionShape>()) {

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
            if (!node->HasComponent<RigidBody>())
            {
                PODVector<NewtonCollisionShape*> compsOnNode;
                node->GetDerivedComponents(compsOnNode, false, true);
                colList += compsOnNode;
            }          
        }

        //add to final list
        colShapes += colList;

        if (recurse) {
            for (NewtonCollisionShape* col : colList)
            {
                if(col->GetNode() != startingNode_)
                    GetAloneCollisionShapes(colShapes, col->GetNode(), false, true);
            }
        }
    }









    void OnPhysicsNodeAdded(Node* node)
    {
        //trigger a rebuild on the root of the new tree.
        PODVector<RigidBody*> rigBodies;
        GetRootRigidBodies(rigBodies, node, false);
        if (rigBodies.Size()) {
            RigidBody* mostRootRigBody = rigBodies.Back();
            if (mostRootRigBody)
                mostRootRigBody->MarkDirty(true);
        }
    }

    void OnPhysicsNodeRemoved(Node* oldParent)
    {
        //trigger a rebuild on the root of the old tree.
        PODVector<RigidBody*> rigBodies;
        GetRootRigidBodies(rigBodies, oldParent, false);
        if (rigBodies.Size()) {
            RigidBody* mostRootRigBody = rigBodies.Back();
            if (mostRootRigBody)
                mostRootRigBody->MarkDirty(true);
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
        NewtonCollisionShape_HeightmapTerrain::RegisterObject(context);

        RigidBody::RegisterObject(context);
        NewtonMeshObject::RegisterObject(context);
        NewtonConstraint::RegisterObject(context);
        NewtonFixedDistanceConstraint::RegisterObject(context);
        NewtonBallAndSocketConstraint::RegisterObject(context);
        NewtonKinematicsControllerConstraint::RegisterObject(context);
        //Constraint::RegisterObject(context);
        
        //RaycastVehicle::RegisterObject(context);

        NewtonPhysicsMaterial::RegisterObject(context);
        NewtonPhysicsMaterialContactPair::RegisterObject(context);
    }








}
