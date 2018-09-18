#pragma once



#include "../Scene/Component.h"
#include "Newton.h"
#include "../Container/Vector.h"
class NewtonWorld;
class dMatrix;
class dCustomJoint;
namespace Urho3D
{
    class Component;
    class CollisionShape;
    class RigidBody;
    class Constraint;
    class PhysicsMaterial;
    class PhysicsMaterialContactPair;
    class Sphere;
    class BoundingBox;
    class NewtonMeshObject;
    class Context;

    static const Vector3 DEF_GRAVITY = Vector3(0, -9.81, 0);
    static const String DEF_PHYSICS_CATEGORY = "Physics";


    class URHO3D_API  RigidBodyContactEntry : public Object
    {
        URHO3D_OBJECT(RigidBodyContactEntry, Object);
    public:

        RigidBodyContactEntry(Context* context);
        virtual ~RigidBodyContactEntry() override;

        /// Register object factory.
        static void RegisterObject(Context* context);

        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest);

        unsigned int hashKey_ = 0;

        WeakPtr<RigidBody> body0 = nullptr;
        WeakPtr<RigidBody> body1 = nullptr;
        bool wakeFlag_ = false;
        bool wakeFlagPrev_ = false;
        bool inContact_ = false;
        int numContacts = 0;
        PODVector<Vector3> contactPositions; //global space
        PODVector<Vector3> contactNormals;   //normal relative to body0
    };


    class URHO3D_API PhysicsWorld : public Component
    {
        URHO3D_OBJECT(PhysicsWorld, Component);
    public:

        friend class CollisionShape;
        friend class CollisionShape_Geometry;
        friend class CollisionShape_ConvexDecompositionCompound;
        friend class NewtonCollisionShape_SceneCollision;
        friend class RigidBody;
        friend class Constraint;

        /// Construct.
        PhysicsWorld(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~PhysicsWorld() override;
        /// Register object factory.
        static void RegisterObject(Context* context);

        /// Return the internal Newton world.
        NewtonWorld* GetNewtonWorld() { return newtonWorld_; }


        /// Saves the NewtonWorld to a serializable newton file.
        void SerializeNewtonWorld(String fileName);

        /// Return a name for the currently used speed plugin (SSE, AVX, AVX2)
        String GetSolverPluginName();






        /// Return rigid bodies by a sphere query.
        void GetRigidBodies(PODVector<RigidBody*>& result, const Sphere& sphere, unsigned collisionMask = M_MAX_UNSIGNED);
        /// Return rigid bodies by a box query.
        void GetRigidBodies(PODVector<RigidBody*>& result, const BoundingBox& box, unsigned collisionMask = M_MAX_UNSIGNED);
        /// Return rigid bodies by contact test with the specified body.
        void GetRigidBodies(PODVector<RigidBody*>& result, const RigidBody* body);



        RigidBodyContactEntry* GetCreateBodyContactEntry(unsigned int key);




        
        ///set the global force acting on all rigid bodies in the world
        void SetGravity(const Vector3& force);
        ///return global force acting on all rigid bodies
        Vector3 GetGravity();

        ///set the physics scale of the world
        void SetPhysicsScale(float scale) { physicsScale_ = scale; }

        Matrix3x4 GetPhysicsWorldFrame() const { return Matrix3x4(Vector3::ZERO, Quaternion::IDENTITY, physicsScale_); }

        float GetPhysicsScale() const {
            return physicsScale_;
        }

        /// set how many iterations newton will run.
        void SetIterationCount(int numIterations);

        int GetIterationCount() const;
        /// set how many substeps newton will run per iteration (slightly different effect that changing Iteration Count)
        void SetSubstepCount(int numSubsteps);

        int GetSubstepCount() const;
        /// set how many threads the newton can use.
        void SetThreadCount(int numThreads);

        int GetThreadCount() const;

        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:


        ///Global force
        Vector3 gravity_ = DEF_GRAVITY;

        /// number of thread to allow newton to use
        int newtonThreadCount_ = 4;
        /// number of iterations newton will internally use.
        int iterationCount_ = 4;
        /// number of substeps per iteration.
        int numSubsteps_ = 2;


        virtual void OnSceneSet(Scene* scene) override;

        void addCollisionShape(CollisionShape* collision);
        void removeCollisionShape(CollisionShape* collision);

        void addRigidBody(RigidBody* body);
        void removeRigidBody(RigidBody* body);

        void addConstraint(Constraint* constraint);
        void removeConstraint(Constraint* constraint);


        void addPhysicsMaterial(PhysicsMaterial* material);
        void computeMaterialPairs();

        Vector<WeakPtr<CollisionShape>> collisionComponentList;
        Vector<WeakPtr<RigidBody>> rigidBodyComponentList;
        Vector<WeakPtr<Constraint>> constraintList;
        Vector<SharedPtr<PhysicsMaterial>> physMaterialList;
        Vector<SharedPtr<PhysicsMaterialContactPair>> physMaterialPairList;


        void freeWorld();

        void addToFreeQueue(NewtonBody* newtonBody);
        void addToFreeQueue(dCustomJoint* newtonConstraint);
        void addToFreeQueue(NewtonCollision* newtonCollision);


        PODVector<NewtonBody*> freeBodyQueue_;
        PODVector<dCustomJoint*> freeConstraintQueue_;
        PODVector<NewtonCollision*> freeCollisionQueue_;


        void applyNewtonWorldSettings();


        HashMap<unsigned int, SharedPtr<RigidBodyContactEntry>> bodyContactMap_;
        
        void parseBodyContactMap();
        bool contactMapLocked_ = false;

        /// Step the simulation forward.
        void HandleUpdate(StringHash eventType, VariantMap& eventData);
        void rebuildDirtyPhysicsComponents();


        /// Internal newton world
        NewtonWorld* newtonWorld_ = nullptr;

        RigidBody* sceneBody_ = nullptr;

        SharedPtr<PhysicsMaterial> defaultPhysicsMaterial_ = nullptr;

        float physicsScale_ = 0.25f;

        ///convex casts
        static const int convexCastRetInfoSize_ = 1000;
        NewtonWorldConvexCastReturnInfo convexCastRetInfoArray[convexCastRetInfoSize_];
        int DoNewtonCollideTest(const float* const matrix, const NewtonCollision* shape);
        void GetBodiesInConvexCast(PODVector<RigidBody*>& result, int numContacts);

        ///newton mesh caching
        HashMap<StringHash, SharedPtr<NewtonMeshObject>> newtonMeshCache_;

        ///returns a unique key for looking up an exising NewtonMesh from the cache.
        static StringHash NewtonMeshKey(String modelResourceName, int modelLodLevel, String otherData);
        NewtonMeshObject* GetCreateNewtonMesh(StringHash urhoNewtonMeshKey);
        NewtonMeshObject* GetNewtonMesh(StringHash urhoNewtonMeshKey);
        




private:
    void freePhysicsInternals();
};



    String NewtonThreadProfilerString(int threadIndex);

    /// netwon body callbacks
    void Newton_ApplyForceAndTorqueCallback(const NewtonBody* body, dFloat timestep, int threadIndex);
    void Newton_SetTransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);
    void Newton_DestroyBodyCallback(const NewtonBody* body);
    unsigned Newton_WorldRayPrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData);

    /// newton material callbacks
    void Newton_ProcessContactsCallback(const NewtonJoint* contactJoint, dFloat timestep, int threadIndex);
    int Newton_AABBOverlapCallback(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex);

    void URHO3D_API GetRootRigidBodies(PODVector<RigidBody*>& rigidBodies, Node* node, bool includeScene);
    void URHO3D_API GetNextChildRigidBodies(PODVector<RigidBody*>& rigidBodies, Node* node);
    void URHO3D_API GetAloneCollisionShapes(PODVector<CollisionShape*>& colShapes, Node* startingNode, bool includeStartingNodeShapes);

    void URHO3D_API RebuildPhysicsNodeTree(Node* node);


    /// Register Physics library objects.
    void URHO3D_API RegisterPhysicsLibrary(Context* context);
}
