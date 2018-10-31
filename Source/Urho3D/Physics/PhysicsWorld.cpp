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
#include "BallAndSocketConstraint.h"
#include "NewtonKinematicsJoint.h"
#include "NewtonDebugDrawing.h"
#include "dgMatrix.h"
#include "dCustomJoint.h"
#include "Graphics/DebugRenderer.h"
#include "FullyFixedConstraint.h"
#include "HingeConstraint.h"
#include "Scene/SceneEvents.h"
#include "SliderConstraint.h"
#include "6DOFConstraint.h"
#include "../dVehicle/dVehicleManager.h"
#include "PhysicsVehicle.h"
#include "VehicleTire.h"
#include "Engine/Engine.h"

namespace Urho3D {


    static const Vector3 DEFAULT_GRAVITY = Vector3(0.0f, -9.81f, 0.0f);

    PhysicsWorld::PhysicsWorld(Context* context) : Component(context)
    {

        SubscribeToEvent(E_SCENESUBSYSTEMUPDATE, URHO3D_HANDLER(PhysicsWorld, HandleSceneUpdate));
    }

    PhysicsWorld::~PhysicsWorld()
    {
    }

    void PhysicsWorld::RegisterObject(Context* context)
    {
        context->RegisterFactory<PhysicsWorld>(DEF_PHYSICS_CATEGORY.CString());
        URHO3D_COPY_BASE_ATTRIBUTES(Component);
        URHO3D_ACCESSOR_ATTRIBUTE("Gravity", GetGravity, SetGravity, Vector3, DEFAULT_GRAVITY, AM_DEFAULT);
        URHO3D_ACCESSOR_ATTRIBUTE("Physics Scale", GetPhysicsScale, SetPhysicsScale, float, 1.0f, AM_DEFAULT);
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



    
    void PhysicsWorld::RayCast(PODVector<PhysicsRayCastIntersection>& intersections, const Ray& ray, float maxDistance, unsigned maxIntersections, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {
        RayCast( intersections, ray.origin_, ray.origin_ + ray.direction_*maxDistance, maxIntersections, collisionMask);
    }

    void PhysicsWorld::RayCast(PODVector<PhysicsRayCastIntersection>& intersections, const Vector3& pointOrigin, const Vector3& pointDestination, unsigned maxIntersections /*= M_MAX_UNSIGNED*/, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {
        PhysicsRayCastUserData data;

        Vector3 origin = SceneToPhysics_Domain(pointOrigin);
        Vector3 destination = SceneToPhysics_Domain(pointDestination);

        NewtonWorldRayCast(newtonWorld_, &UrhoToNewton(origin)[0], &UrhoToNewton(destination)[0], Newton_WorldRayCastFilterCallback, &data, NULL, 0);

        //sort the intersections by distance. we do this because the order that you get is based off bounding box intersection and that is not nessecarily the same of surface intersection order.
        if (data.intersections.Size() > 1)
            Sort(data.intersections.Begin(), data.intersections.End(), PhysicsRayCastIntersectionCompare);


        int intersectCount = 0;
        for (PhysicsRayCastIntersection& intersection : data.intersections)
        {

            unsigned collisionLayerAsBit = CollisionLayerAsBit(intersection.rigBody->GetCollisionLayer());


            if ((intersectCount <= maxIntersections)
                && (collisionLayerAsBit & collisionMask)) {


                intersection.rayIntersectWorldPosition = PhysicsToScene_Domain(intersection.rayIntersectWorldPosition);
                intersection.rayOriginWorld = pointOrigin;
                intersection.rayDistance = (intersection.rayIntersectWorldPosition - pointOrigin).Length();

                intersections += intersection;
                intersectCount++;

            }
        }

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


    Urho3D::Vector3 PhysicsWorld::GetGravity() const
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

    void PhysicsWorld::SetSubstepFactor(int factor)
    {
        subStepFactor = factor;
        applyNewtonWorldSettings();
    }

    int PhysicsWorld::GetSubstepFactor() const
    {
        return subStepFactor;
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

            //draw debug geometry on vehicles
            for (PhysicsVehicle* vehicle : vehicleList) {
                vehicle->DrawDebugGeometry(debug, depthTest);
            }
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


                NewtonMaterialSetCollisionCallback(newtonWorld_, 0, 0, Newton_AABBOverlapCallback, Newton_ProcessContactsCallback);
                //NewtonMaterialSetCompoundCollisionCallback(newtonWorld_, 0, 0, Newton_AABBCompoundOverlapCallback);



                //make the vehicle manager
                vehicleManager_ = new dVehicleManager(newtonWorld_);

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


    void PhysicsWorld::addVehicle(PhysicsVehicle* vehicle)
    {
        vehicleList.Insert(0, WeakPtr<PhysicsVehicle>(vehicle));
    }

    void PhysicsWorld::removeVehicle(PhysicsVehicle* vehicle)
    {
        vehicleList.Remove(WeakPtr<PhysicsVehicle>(vehicle));
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


        vehicleList.Clear();

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
        NewtonSetSolverIterations(newtonWorld_, iterationCount_);
        NewtonSetNumberOfSubsteps(newtonWorld_, 1);
        NewtonSetThreadsCount(newtonWorld_, newtonThreadCount_);
        NewtonSelectBroadphaseAlgorithm(newtonWorld_, 1);//persistent broadphase.

    }

    void PhysicsWorld::formContacts()
{

        for (RigidBody* rigBody : rigidBodyComponentList)
        {
            NewtonBody* newtonBody = rigBody->GetNewtonBody();
            if(!newtonBody)
                continue;


            NewtonJoint* curJoint = NewtonBodyGetFirstContactJoint(newtonBody);
            while (curJoint) {


                NewtonBody* body0 = NewtonJointGetBody0(curJoint);
                NewtonBody* body1 = NewtonJointGetBody1(curJoint);

                RigidBody* rigBody0 = (RigidBody*)NewtonBodyGetUserData(body0);
                RigidBody* rigBody1 = (RigidBody*)NewtonBodyGetUserData(body1);

                if (!rigBody0 || !rigBody1)
                {
                    curJoint = NewtonBodyGetNextContactJoint(newtonBody, curJoint);
                    continue;
                }



                unsigned int key = IntVector2(rigBody0->GetID(), rigBody1->GetID()).ToHash();
                SharedPtr<RigidBodyContactEntry> contactEntry = nullptr;



                contactEntry = GetCreateBodyContactEntry(key);


                
                contactEntry->body0 = rigBody0;
                contactEntry->body1 = rigBody1;


                contactEntry->wakeFlag_ = true;
                contactEntry->inContact_ = NewtonJointIsActive(curJoint);
                contactEntry->numContacts = NewtonContactJointGetContactCount(curJoint);

                if (contactEntry->numContacts > DEF_PHYSICS_MAX_CONTACT_POINTS)
                {
                    contactEntry->ResizeBuffers(contactEntry->numContacts);
                }




                int contactIdx = 0;
                for (void* contact = NewtonContactJointGetFirstContact(curJoint); contact; contact = NewtonContactJointGetNextContact(curJoint, contact))
                {
                    
                    NewtonMaterial* const material = NewtonContactGetMaterial(contact);

                    NewtonCollision* shape0 = NewtonMaterialGetBodyCollidingShape(material, body0);
                    NewtonCollision* shape1 = NewtonMaterialGetBodyCollidingShape(material, body1);


                    CollisionShape* colShape0 = static_cast<CollisionShape*>(NewtonCollisionGetUserData(shape0));
                    CollisionShape* colShape1 = static_cast<CollisionShape*>(NewtonCollisionGetUserData(shape1));




                    //get contact geometric info for the contact struct
                    dVector pos, force, norm, tan0, tan1;
                    NewtonMaterialGetContactPositionAndNormal(material, body0, &pos[0], &norm[0]);
                    NewtonMaterialGetContactTangentDirections(material, body0, &tan0[0], &tan1[0]);
                    NewtonMaterialGetContactForce(material, body0, &force[0]);


                    contactEntry->contactNormals[contactIdx] = PhysicsToScene_Domain(NewtonToUrhoVec3(norm));
                    contactEntry->contactPositions[contactIdx] = PhysicsToScene_Domain(NewtonToUrhoVec3(pos));
                    contactEntry->contactTangent0[contactIdx] = PhysicsToScene_Domain(NewtonToUrhoVec3(tan0));
                    contactEntry->contactTangent1[contactIdx] = PhysicsToScene_Domain(NewtonToUrhoVec3(tan1));
                    contactEntry->contactForces[contactIdx] = PhysicsToScene_Domain(NewtonToUrhoVec3(force));


                    contactEntry->shapes0[contactIdx] = colShape0;
                    contactEntry->shapes1[contactIdx] = colShape1;

                    contactIdx++;
                }
                
                curJoint = NewtonBodyGetNextContactJoint(newtonBody, curJoint);
            }

        }









    }

    void PhysicsWorld::ParseContacts()
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
            else if (!it->second_->inContact_)
            {
                removeKeys += it->second_->hashKey_;
            }
            else if (it->second_->wakeFlag_ && !it->second_->wakeFlagPrev_)//begin contact
            {
                it->second_->inContact_ = true;
                if (it->second_->body0->collisionEventMode_ && it->second_->body1->collisionEventMode_) {
                    SendEvent(E_PHYSICSCOLLISIONSTART, eventData);
                }



                if (it->second_->body0->collisionEventMode_) {
                    if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break; //it is possible someone deleted a body in the previous event.

                    eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body1->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body1;
                    it->second_->body0->GetNode()->SendEvent(E_NODECOLLISIONSTART, eventData);
                }


                if (it->second_->body1->collisionEventMode_) {
                    if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;

                    eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body0->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body0;
                    it->second_->body1->GetNode()->SendEvent(E_NODECOLLISIONSTART, eventData);
                }


                if (it->second_->body0->collisionEventMode_ && it->second_->body1->collisionEventMode_) {
                    if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;

                    //also send the E_NODECOLLISION event
                    SendEvent(E_PHYSICSCOLLISION, eventData);
                }

                if (it->second_->body0->collisionEventMode_) {

                    if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;

                    eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body1->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body1;
                    it->second_->body0->GetNode()->SendEvent(E_NODECOLLISION, eventData);
                }


                if (it->second_->body1->collisionEventMode_) {
                    if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;

                    eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body0->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body0;
                    it->second_->body1->GetNode()->SendEvent(E_NODECOLLISION, eventData);
                }


            }
            else if (!it->second_->wakeFlag_ && it->second_->wakeFlagPrev_)//end contact
            {
                it->second_->inContact_ = false;
                if (it->second_->body0->collisionEventMode_ && it->second_->body1->collisionEventMode_) {
                    SendEvent(E_PHYSICSCOLLISIONEND, eventData);
                }


                if (it->second_->body0->collisionEventMode_) {

                    if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;
                    eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body1->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body1;
                    it->second_->body0->GetNode()->SendEvent(E_NODECOLLISIONEND, eventData);
                }

                if (it->second_->body1->collisionEventMode_) {
                    if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;
                    eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body0->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body0;
                    it->second_->body1->GetNode()->SendEvent(E_NODECOLLISIONEND, eventData);
                }
            }
            else if (it->second_->wakeFlag_ && it->second_->wakeFlagPrev_)//continued contact
            {
                if (it->second_->body0->collisionEventMode_ && it->second_->body1->collisionEventMode_) {
                    SendEvent(E_PHYSICSCOLLISION, eventData);
                }



                if (it->second_->body0->collisionEventMode_) {
                    if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;

                    eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body1->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body1;
                    it->second_->body0->GetNode()->SendEvent(E_NODECOLLISION, eventData);
                }

                if (it->second_->body1->collisionEventMode_) {

                    if (!it->second_->body0.Refs() || !it->second_->body1.Refs()) break;

                    eventData[NodeCollisionStart::P_OTHERNODE] = it->second_->body0->GetNode();
                    eventData[NodeCollisionStart::P_OTHERBODY] = it->second_->body0;
                    it->second_->body1->GetNode()->SendEvent(E_NODECOLLISION, eventData);
                }
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




    void PhysicsWorld::HandleSceneUpdate(StringHash eventType, VariantMap& eventData)
    {
       sceneUpdated_ = true;

       Update(GSS<Engine>()->GetUpdateTimeGoalMs() / 1000.0f / float(subStepFactor), true);
       for(int i = 0; i < subStepFactor-1; i++)
            Update(GSS<Engine>()->GetUpdateTimeGoalMs() / 1000.0f / float(subStepFactor), false);
    }



    void PhysicsWorld::Update(float timestep, bool isRootUpdate)
    {

        URHO3D_PROFILE_FUNCTION();
        float timeStep = timestep*GetScene()->GetTimeScale();
        bool rootRate = isRootUpdate;

        if (rootRate) {

            //Resolve the root rigid body component if it needs created:
            if (!sceneBody_) {
                sceneBody_ = GetScene()->GetOrCreateComponent<RigidBody>();
                sceneBody_->SetIsSceneRootBody(true);
            }
        }



        if (simulationStarted_) {
            URHO3D_PROFILE("Wait For ASync Update To finish.");
            NewtonWaitForUpdateToFinish(newtonWorld_);
        }
        VariantMap sendEventData;
        sendEventData[PhysicsPostStep::P_WORLD] = this;
        sendEventData[PhysicsPostStep::P_TIMESTEP] = timeStep;
        if (rootRate) {

            if (simulationStarted_) {
                //send post physics event.
                SendEvent(E_PHYSICSPOSTSTEP, sendEventData);
                {
                    URHO3D_PROFILE("Apply Node Transforms");

                    {
                        URHO3D_PROFILE("Rigid Body Order Pre Sort");
                        //sort the rigidBodyComponentList by scene depth.
                        if (rigidBodyListNeedsSorted) {
                            Sort(rigidBodyComponentList.Begin(), rigidBodyComponentList.End(), RigidBodySceneDepthCompare);
                            rigidBodyListNeedsSorted = false;
                        }
                    }

                    //apply the transform of all rigid body components to their respective nodes.
                    for (RigidBody* rigBody : rigidBodyComponentList)
                    {
                        if (rigBody->GetInternalTransformDirty()) {
                            rigBody->ApplyTransform(timeStep);


                            if (rigBody->InterpolationWithinRestTolerance())
                                rigBody->MarkInternalTransformDirty(false);
                        }
                    }

                    //tell vehilces to update the nodes for the tires.
                    for (PhysicsVehicle* vehicle : vehicleList)
                    {
                        vehicle->applyTransforms();
                    }

                }
            }






        }

        formContacts();

        if (rootRate) {
            ParseContacts();

            freePhysicsInternals();

            //rebuild stuff.
            rebuildDirtyPhysicsComponents();
        }


        SendEvent(E_PHYSICSPRESTEP, sendEventData);
        {
            URHO3D_PROFILE("NewtonUpdate");
            //use target time step to give newton constant time steps. 


            NewtonUpdateAsync(newtonWorld_, timeStep);
            simulationStarted_ = true;
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


            //cross check if any vehicles are using the body - if so mark the vehicle dirty to because it will need rebuilt.
            for (PhysicsVehicle* vehicle : vehicleList)
            {
                if (vehicle->rigidBody_ == rigBody) {
                    vehicle->MarkDirty();
                }
            }




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

        //rebuild vehicles if they need rebuilt
        for (PhysicsVehicle* vehicle : vehicleList)
        {
            if (vehicle->isDirty_) {
                vehicle->reBuild();
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


    URHO3D_API RigidBody*  GetRigidBody(Node* node, bool includeScene)
    {
        Node* curNode = node;

        while (curNode) {
            if (curNode == curNode->GetScene() && !includeScene)
            {
                return nullptr;
            }

            RigidBody* body = curNode->GetComponent<RigidBody>();

            if (body)
                return body;

            curNode = curNode->GetParent();
        }

        return nullptr;
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
            startingNode->GetDerivedComponents<CollisionShape>(colShapes, false, false);
        }



        PODVector<Node*> immediateChildren;
        startingNode->GetChildren(immediateChildren, false);

        for (Node* child : immediateChildren) {
            if (child->HasComponent<RigidBody>())
                continue;
            else
            {
                child->GetDerivedComponents<CollisionShape>(colShapes, false, false);
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


    unsigned CollisionLayerAsBit(unsigned layer)
    {
        if (layer == 0)
            return M_MAX_UNSIGNED;

        return (0x1 << layer - 1);
    }

    void RegisterPhysicsLibrary(Context* context)
    {
        PhysicsWorld::RegisterObject(context);

        CollisionShape::RegisterObject(context);
        CollisionShape_Box::RegisterObject(context);
        CollisionShape_Sphere::RegisterObject(context);
        CollisionShape_Cylinder::RegisterObject(context);
        CollisionShape_ChamferCylinder::RegisterObject(context);
        CollisionShape_Capsule::RegisterObject(context);
        CollisionShape_Cone::RegisterObject(context);
        CollisionShape_Geometry::RegisterObject(context);
        CollisionShape_ConvexHull::RegisterObject(context);
        CollisionShape_ConvexHullCompound::RegisterObject(context);
        CollisionShape_ConvexDecompositionCompound::RegisterObject(context);
        CollisionShape_TreeCollision::RegisterObject(context);
        CollisionShape_HeightmapTerrain::RegisterObject(context);

        RigidBody::RegisterObject(context);
        NewtonMeshObject::RegisterObject(context);
        Constraint::RegisterObject(context);
        FixedDistanceConstraint::RegisterObject(context);
        BallAndSocketConstraint::RegisterObject(context);
        SixDof_Constraint::RegisterObject(context);
        HingeConstraint::RegisterObject(context);
        SliderConstraint::RegisterObject(context);
        FullyFixedConstraint::RegisterObject(context);
        KinematicsControllerConstraint::RegisterObject(context);
        RigidBodyContactEntry::RegisterObject(context);

        PhysicsVehicle::RegisterObject(context);
        VehicleTire::RegisterObject(context);

    }












    RigidBodyContactEntry::RigidBodyContactEntry(Context* context) : Object(context)
    {
        ResizeBuffers(DEF_PHYSICS_MAX_CONTACT_POINTS);
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
                debug->AddLine(contactPositions[i], (contactPositions[i] + contactNormals[i]), Color::GREEN, depthTest);
            }
        }
    }

    void RigidBodyContactEntry::ResizeBuffers(int size)
    {
        contactForces.Resize(size);
        contactPositions.Resize(size);
        contactNormals.Resize(size);
        contactTangent0.Resize(size);
        contactTangent1.Resize(size);
        shapes0.Resize(size);
        shapes1.Resize(size);
    }

}
