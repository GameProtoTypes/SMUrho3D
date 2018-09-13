#include "PhysicsWorld.h"
#include "IO/Log.h"
#include "CollisionShape.h"
#include "CollisionShapesDerived.h"
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
#include "Constraint.h"
#include "FixedDistanceConstraint.h"
#include "Core/Profiler.h"
#include "PhysicsEvents.h"
#include "Graphics/VisualDebugger.h"
#include "PhysicsMaterial.h"
#include "BallAndSocketConstraint.h"
#include "NewtonKinematicsJoint.h"
#include "NewtonDebugDrawing.h"
#include "dgMatrix.h"
#include "dCustomJoint.h"
#include "Graphics/DebugRenderer.h"
#include "FullyFixedConstraint.h"
#include "HingeConstraint.h"

namespace Urho3D {


    static const Vector3 DEFAULT_GRAVITY = Vector3(0.0f, -9.81f, 0.0f);

    PhysicsWorld::PhysicsWorld(Context* context) : Component(context)
    {
        SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(PhysicsWorld, HandleUpdate));

    }

    PhysicsWorld::~PhysicsWorld()
    {
    }

    void PhysicsWorld::RegisterObject(Context* context)
    {
        context->RegisterFactory<PhysicsWorld>(DEF_PHYSICS_CATEGORY.CString());
    }




    void PhysicsWorld::SerializeNewtonWorld(String fileName)
    {
        NewtonSerializeToFile(newtonWorld_, fileName.CString(), nullptr, nullptr);
    }



    
    String PhysicsWorld::GetSolverPluginName()
    {
        void* plugin = NewtonCurrentPlugin(newtonWorld_);
        return String(NewtonGetPluginString(newtonWorld_, plugin));
    }



    Urho3D::RigidBodyContactEntry* PhysicsWorld::GetCreateBodyContactEntry(unsigned int key)
    {
        if (!bodyContactMap_.Contains(key)) {
            SharedPtr<RigidBodyContactEntry> newContact = context_->CreateObject<RigidBodyContactEntry>();
            newContact->hashKey_ = key;
            bodyContactMap_.Insert(Pair<unsigned int, SharedPtr<RigidBodyContactEntry>>(key, newContact));

        }

        return bodyContactMap_[key];
    }

    void PhysicsWorld::SetGravity(const Vector3& force)
    {
        gravity_ = force;
    }


    Urho3D::Vector3 PhysicsWorld::GetGravity()
{
        return gravity_;
    }

    void PhysicsWorld::SetIterationCount(int numIterations /*= 8*/)
    {
        iterationCount_ = numIterations;
        applyNewtonWorldSettings();
    }


    int PhysicsWorld::GetIterationCount() const
    {
        return iterationCount_;
    }

    void PhysicsWorld::SetSubstepCount(int numSubsteps)
    {
        numSubsteps_ = numSubsteps;
        applyNewtonWorldSettings();
    }

    int PhysicsWorld::GetSubstepCount() const
    {
        return numSubsteps_;
    }

    void PhysicsWorld::SetThreadCount(int numThreads)
    {
        newtonThreadCount_ = numThreads;
        applyNewtonWorldSettings();
    }

    int PhysicsWorld::GetThreadCount() const
    {
        return newtonThreadCount_;
    }

    void PhysicsWorld::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        URHO3D_PROFILE_FUNCTION();
        if (debug)
        {
            //draw debug geometry on constraints.
            for (Constraint* constraint : constraintList)
            {
                constraint->DrawDebugGeometry(debug, depthTest);
            }

            //draw debug geometry for contacts
            for (HashMap<unsigned int, SharedPtr<RigidBodyContactEntry>>::Iterator i = bodyContactMap_.Begin(); i != bodyContactMap_.End(); ++i)
            {
               
                   i->second_->DrawDebugGeometry(debug, depthTest);
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

    void PhysicsWorld::OnSceneSet(Scene* scene)
    {
        if (scene) {
            //create the newton world
            if (newtonWorld_ == nullptr) {
                newtonWorld_ = NewtonCreate();
                NewtonWorldSetUserData(newtonWorld_, (void*)this);
                applyNewtonWorldSettings();

                sceneBody_ = scene->CreateComponent<RigidBody>();
                sceneBody_->SetIsSceneRootBody(true);

                //add a default physics material
                defaultPhysicsMaterial_ = context_->CreateObject<PhysicsMaterial>();
                addPhysicsMaterial(defaultPhysicsMaterial_);


            }
        }
        else
        {
            //wait for update to finish if in async mode so we can safely clean up.
            NewtonWaitForUpdateToFinish(newtonWorld_);

            freeWorld();
        }
    }

    void PhysicsWorld::addCollisionShape(CollisionShape* collision)
    {
        collisionComponentList.Insert(0, WeakPtr<CollisionShape>(collision));
    }

    void PhysicsWorld::removeCollisionShape(CollisionShape* collision)
    {
        collisionComponentList.Remove(WeakPtr<CollisionShape>(collision));
    }

    void PhysicsWorld::addRigidBody(RigidBody* body)
    {
        rigidBodyComponentList.Insert(0, WeakPtr<RigidBody>(body));
    }

    void PhysicsWorld::removeRigidBody(RigidBody* body)
    {
        rigidBodyComponentList.Remove(WeakPtr<RigidBody>(body));
    }

    void PhysicsWorld::addConstraint(Constraint* constraint)
    {
        constraintList.Insert(0, WeakPtr<Constraint>(constraint));
    }

    void PhysicsWorld::removeConstraint(Constraint* constraint)
    {
        constraintList.Remove(WeakPtr<Constraint>(constraint));
    }

    void PhysicsWorld::addPhysicsMaterial(PhysicsMaterial* material)
    {
        if (physMaterialList.Contains(SharedPtr<PhysicsMaterial>(material)))
            return;

        else
        {
            physMaterialList.Insert(0, SharedPtr<PhysicsMaterial>(material));
            material->newtonGroupId = NewtonMaterialCreateGroupID(newtonWorld_);

            computeMaterialPairs();
        }
    }

    void PhysicsWorld::computeMaterialPairs()
    {
        physMaterialPairList.Clear();
        for (PhysicsMaterial* mat1 : physMaterialList) {
            for (PhysicsMaterial* mat2 : physMaterialList) {
                SharedPtr<PhysicsMaterialContactPair> newPair = context_->CreateObject<PhysicsMaterialContactPair>();
                newPair->SetMaterials(mat1, mat2);//compute.

                NewtonMaterialSetCallbackUserData(newtonWorld_, mat1->newtonGroupId, mat2->newtonGroupId, (void*)newPair);
                NewtonMaterialSetCollisionCallback(newtonWorld_, newPair->newtonGroupId0, newPair->newtonGroupId1, Newton_AABBOverlapCallback, Newton_ProcessContactsCallback);
                

                physMaterialPairList.Insert(0, newPair);
            }
        }
    }

    void PhysicsWorld::freeWorld()
    {




        //free any joints
        for (Constraint* constraint : constraintList)
        {
            constraint->freeInternal();
        }
        constraintList.Clear();

        //free any collision shapes currently in the list
        for (CollisionShape* col : collisionComponentList)
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

        //free the actual memory
        freePhysicsInternals();


        //destroy newton world.
        if (newtonWorld_ != nullptr) {
            NewtonDestroy(newtonWorld_);
            newtonWorld_ = nullptr;
        }
    }




    void PhysicsWorld::addToFreeQueue(NewtonBody* newtonBody)
    {
        freeBodyQueue_ += newtonBody;
    }

    void PhysicsWorld::addToFreeQueue(dCustomJoint* newtonConstraint)
    {
        freeConstraintQueue_ += newtonConstraint;
    }

    void PhysicsWorld::addToFreeQueue(NewtonCollision* newtonCollision)
    {
        freeCollisionQueue_ += newtonCollision;
    }

    void PhysicsWorld::applyNewtonWorldSettings()
    {
        NewtonSetSolverModel(newtonWorld_, iterationCount_);
        NewtonSetNumberOfSubsteps(newtonWorld_, numSubsteps_);
        NewtonSetThreadsCount(newtonWorld_, newtonThreadCount_);
        NewtonSelectBroadphaseAlgorithm(newtonWorld_, 1);//persistent broadphase.
    }

    void PhysicsWorld::parseBodyContactMap()
    {
        PODVector<unsigned int> removeKeys;
        VariantMap eventData;
        eventData[PhysicsCollisionStart::P_WORLD] = this;
        for (HashMap<unsigned int, SharedPtr<RigidBodyContactEntry>>::Iterator it = bodyContactMap_.Begin(); it != bodyContactMap_.End(); it++)
        {
            eventData[PhysicsCollisionStart::P_BODYA] = it->second_->body0;
            eventData[PhysicsCollisionStart::P_BODYB] = it->second_->body1;
            
            eventData[PhysicsCollisionStart::P_CONTACT_DATA] = it->second_;

            if (!it->second_->body0.Refs() || !it->second_->body1.Refs())//check expired
            {
                removeKeys += it->second_->hashKey_;
            }
            else if (it->second_->wakeFlag_ && !it->second_->wakeFlagPrev_)//begin contact
            {
                it->second_->inContact_ = true;
                SendEvent(E_PHYSICSCOLLISIONSTART, eventData);

                if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;

                eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body1->GetNode();
                eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body1;
                it->second_->body0->GetNode()->SendEvent(E_NODECOLLISIONSTART, eventData);

                if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;
                eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body0->GetNode();
                eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body0;
                it->second_->body1->GetNode()->SendEvent(E_NODECOLLISIONSTART, eventData);

                if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;
                //also send the E_NODECOLLISION event
                SendEvent(E_PHYSICSCOLLISION, eventData);

                if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;
                eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body1->GetNode();
                eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body1;
                it->second_->body0->GetNode()->SendEvent(E_NODECOLLISION, eventData);

                if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;
                eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body0->GetNode();
                eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body0;
                it->second_->body1->GetNode()->SendEvent(E_NODECOLLISION, eventData);



            }
            else if (!it->second_->wakeFlag_ && it->second_->wakeFlagPrev_)//end contact
            {
                it->second_->inContact_ = false;
                SendEvent(E_PHYSICSCOLLISIONEND, eventData);

                if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;
                eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body1->GetNode();
                eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body1;
                it->second_->body0->GetNode()->SendEvent(E_NODECOLLISIONEND, eventData);

                if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;
                eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body0->GetNode();
                eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body0;
                it->second_->body1->GetNode()->SendEvent(E_NODECOLLISIONEND, eventData);
            }
            else if(it->second_->wakeFlag_ && it->second_->wakeFlagPrev_)//continued contact
            {
                SendEvent(E_PHYSICSCOLLISION, eventData);

                if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;
                eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body1->GetNode();
                eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body1;
                it->second_->body0->GetNode()->SendEvent(E_NODECOLLISION, eventData);

                if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;
                eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body0->GetNode();
                eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body0;
                it->second_->body1->GetNode()->SendEvent(E_NODECOLLISION, eventData);
            }
            else if (!it->second_->wakeFlag_ && !it->second_->wakeFlagPrev_)//no contact for one update. (mark for removal from the map)
            {
               removeKeys += it->second_->hashKey_;
            }

            //move on..
            it->second_->wakeFlagPrev_ = it->second_->wakeFlag_;
            it->second_->wakeFlag_ = false;
        }


        //clean old contacts.
        for (auto key : removeKeys)
        {
            bodyContactMap_.Erase(key);
        }
    }


    void PhysicsWorld::HandleUpdate(StringHash eventType, VariantMap& eventData)
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



        SendEvent(E_PHYSICSPRESTEP, sendEventData);

        //rebuild collision shapes from child nodes to root nodes.
        rebuildDirtyPhysicsComponents();

        parseBodyContactMap();

        freePhysicsInternals();

        {
            URHO3D_PROFILE("NewtonUpdate");
            //use target time step to give newton constant time steps. 

            
            NewtonUpdateAsync(newtonWorld_, timeStep);
           // NewtonWaitForUpdateToFinish(newtonWorld_);

        }



    }



    void PhysicsWorld::rebuildDirtyPhysicsComponents()
    {
        URHO3D_PROFILE_FUNCTION();



        //rebuild dirty collision shapes
        for (CollisionShape* colShape : collisionComponentList)
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


            rigBody->reBuildBody();
            rigBody->MarkDirty(false);
        }

        //apply deferred actions (like impulses/velocity sets etc.) that were waiting for a real body to be built.
        for (RigidBody* rigBody : rigidBodyComponentList)
        {
            rigBody->applyDefferedActions();
        }



        //rebuild contraints if they need rebuilt (dirty)
        for (Constraint* constraint : constraintList)
        {
            if (constraint->needsRebuilt_)
                constraint->reEvalConstraint();
        }
    }


    int PhysicsWorld::DoNewtonCollideTest(const float* const matrix, const NewtonCollision* shape)
    {

        return  NewtonWorldCollide(newtonWorld_,
            matrix, shape, nullptr,
            Newton_WorldRayPrefilterCallback, convexCastRetInfoArray,
            convexCastRetInfoSize_, 0);

    }

    void PhysicsWorld::GetBodiesInConvexCast(PODVector<RigidBody*>& result, int numContacts)
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

    Urho3D::StringHash PhysicsWorld::NewtonMeshKey(String modelResourceName, int modelLodLevel, String otherData)
    {
        return modelResourceName + String(modelLodLevel) + otherData;
    }

    NewtonMeshObject* PhysicsWorld::GetCreateNewtonMesh(StringHash urhoNewtonMeshKey)
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

    NewtonMeshObject* PhysicsWorld::GetNewtonMesh(StringHash urhoNewtonMeshKey)
    {
        if (newtonMeshCache_.Contains(urhoNewtonMeshKey)) {
            return newtonMeshCache_[urhoNewtonMeshKey];
        }
        return nullptr;
    }

    void PhysicsWorld::freePhysicsInternals()
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


    //returns first occuring child rigid bodies.
    void URHO3D_API GetNextChildRigidBodies(PODVector<RigidBody*>& rigidBodies, Node* node)
    {

        PODVector<Node*> immediateChildren;
        node->GetChildren(immediateChildren, false);

        for (Node* child : immediateChildren) {
            if (child->HasComponent<RigidBody>())
                rigidBodies += child->GetComponent<RigidBody>();
            else
                GetNextChildRigidBodies(rigidBodies, child);
        }

    }

    //recurses up the scene tree starting a node and continuing up every branch adding collision shapes to the array until a rigid body is encountered in which case the algorithm stops traversing that branch.
    void GetAloneCollisionShapes(PODVector<CollisionShape*>& colShapes, Node* startingNode, bool includeStartingNodeShapes)
    {

        if (includeStartingNodeShapes)
        {
            CollisionShape* shape = startingNode->GetDerivedComponent<CollisionShape>();
            if (shape) {
                colShapes += shape;
            }
        }



        PODVector<Node*> immediateChildren;
        startingNode->GetChildren(immediateChildren, false);

        for (Node* child : immediateChildren) {
            if (child->HasComponent<RigidBody>())
                return;
            else
            {
                CollisionShape* shape = child->GetDerivedComponent<CollisionShape>();
                if (shape) {
                    colShapes += shape;
                }

                GetAloneCollisionShapes(colShapes, child, false);

            }

        }
    }









    void RebuildPhysicsNodeTree(Node* node)
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


    void RegisterPhysicsLibrary(Context* context)
    {
        PhysicsWorld::RegisterObject(context);

        CollisionShape::RegisterObject(context);
        CollisionShape_Box::RegisterObject(context);
        CollisionShape_Sphere::RegisterObject(context);
        CollisionShape_Cylinder::RegisterObject(context);
        CollisionShape_Capsule::RegisterObject(context);
        CollisionShape_Cone::RegisterObject(context);
        CollisionShape_Geometry::RegisterObject(context);
        CollisionShape_ConvexHull::RegisterObject(context);
        CollisionShape_ConvexHullCompound::RegisterObject(context);
        CollisionShape_ConvexDecompositionCompound::RegisterObject(context);
        NewtonCollisionShape_HeightmapTerrain::RegisterObject(context);

        RigidBody::RegisterObject(context);
        NewtonMeshObject::RegisterObject(context);
        Constraint::RegisterObject(context);
        FixedDistanceConstraint::RegisterObject(context);
        BallAndSocketConstraint::RegisterObject(context);
        HingeConstraint::RegisterObject(context);
        FullyFixedConstraint::RegisterObject(context);
        KinematicsControllerConstraint::RegisterObject(context);
        RigidBodyContactEntry::RegisterObject(context);

        //RaycastVehicle::RegisterObject(context);

        PhysicsMaterial::RegisterObject(context);
        PhysicsMaterialContactPair::RegisterObject(context);
    }












    RigidBodyContactEntry::RigidBodyContactEntry(Context* context) : Object(context)
    {

    }

    RigidBodyContactEntry::~RigidBodyContactEntry()
    {

    }

    void RigidBodyContactEntry::RegisterObject(Context* context)
    {
        context->RegisterFactory<RigidBodyContactEntry>();
    }

    void RigidBodyContactEntry::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        //draw contact points
        if (inContact_)
        {
            for (int i = 0; i < numContacts; i++)
            {

                //debug->AddCross(contactPositions[i], 0.2f, Color::RED, false);
                debug->AddLine(contactPositions[i], contactPositions[i] + contactNormals[i], Color::GREEN, depthTest);

            }





        }


    }

}
