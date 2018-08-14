#pragma once



#include "../Scene/Component.h"
#include "Newton.h"
#include "../Container/Vector.h"
class NewtonWorld;
class dMatrix;
namespace Urho3D
{
    class Component;
    class NewtonCollisionShape;
    class NewtonRigidBody;
    class NewtonConstraint;
    class Sphere;
    class BoundingBox;
    class NewtonMeshObject;

    static const Vector3 DEF_GRAVITY = Vector3(0, -9.81, 0);
    static const String DEF_PHYSICS_CATEGORY = "Physics";



    class URHO3D_API UrhoNewtonPhysicsWorld : public Component
    {
        URHO3D_OBJECT(UrhoNewtonPhysicsWorld, Component);
    public:

        friend class NewtonCollisionShape;
        friend class NewtonCollisionShape_Geometry;
        friend class NewtonCollisionShape_ConvexDecompositionCompound;
        friend class NewtonRigidBody;
        friend class NewtonConstraint;

        /// Construct.
        UrhoNewtonPhysicsWorld(Context* context);
        /// Destruct. Free the rigid body and geometries.
        ~UrhoNewtonPhysicsWorld() override;
        /// Register object factory.
        static void RegisterObject(Context* context);

        /// Return the internal Newton world.
        NewtonWorld* GetNewtonWorld() { return newtonWorld_; }
        /// Saves the NewtonWorld to a serializable newton file.
        void SerializeNewtonWorld(String fileName);

        /// Return a name for the currently used speed plugin (SSE, AVX, AVX2)
        String GetSolverPluginName();


        /// Return rigid bodies by a sphere query.
        void GetRigidBodies(PODVector<NewtonRigidBody*>& result, const Sphere& sphere, unsigned collisionMask = M_MAX_UNSIGNED);
        /// Return rigid bodies by a box query.
        void GetRigidBodies(PODVector<NewtonRigidBody*>& result, const BoundingBox& box, unsigned collisionMask = M_MAX_UNSIGNED);
        /// Return rigid bodies by contact test with the specified body.
        void GetRigidBodies(PODVector<NewtonRigidBody*>& result, const NewtonRigidBody* body);

        
        ///set the global force acting on all rigid bodies in the world
        void SetGravity(const Vector3& force);
        ///return global force acting on all rigid bodies
        Vector3 GetGravity();

        /// set how many iterations newton will run in total per update.
        void SetTotalIterations(int numIterations = 8);



        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:


        ///Global force
        Vector3 gravity_ = DEF_GRAVITY;

        /// number of thread to allow newton to use (#todo draw from thread pool)
        int newtonThreadCount_ = 4;
        /// number of iterations newton will internally use.
        int totalIterationCount_ = 8;


        virtual void OnSceneSet(Scene* scene) override;

        void addCollisionShape(NewtonCollisionShape* collision);
        void removeCollisionShape(NewtonCollisionShape* collision);

        void addRigidBody(NewtonRigidBody* body);
        void removeRigidBody(NewtonRigidBody* body);

        void addConstraint(NewtonConstraint* constraint);
        void removeConstraint(NewtonConstraint* constraint);

        void freeWorld();


        Vector<WeakPtr<NewtonCollisionShape>> collisionComponentList;
        Vector<WeakPtr<NewtonRigidBody>> rigidBodyComponentList;
        Vector<WeakPtr<NewtonConstraint>> constraintList;


        void applyNewtonWorldSettings();


        /// Step the simulation forward.
        void HandleUpdate(StringHash eventType, VariantMap& eventData);
        void rebuildDirtyPhysicsComponents();


        /// Internal newton world
        NewtonWorld* newtonWorld_ = nullptr;

        ///convex casts
        static const int convexCastRetInfoSize_ = 1000;
        NewtonWorldConvexCastReturnInfo convexCastRetInfoArray[convexCastRetInfoSize_];
        int DoNewtonCollideTest(const float* const matrix, const NewtonCollision* shape);
        void GetBodiesInConvexCast(PODVector<NewtonRigidBody*>& result, int numContacts);

        ///newton mesh caching
        HashMap<StringHash, SharedPtr<NewtonMeshObject>> newtonMeshCache_;

        ///returns a unique key for looking up an exising NewtonMesh from the cache.
        static StringHash NewtonMeshKey(String modelResourceName, int modelLodLevel, String otherData);
        NewtonMeshObject* GetCreateNewtonMesh(StringHash urhoNewtonMeshKey);
        NewtonMeshObject* GetNewtonMesh(StringHash urhoNewtonMeshKey);
        

    };

    /// netwon callbacks
    void Newton_ApplyForceAndTorqueCallback(const NewtonBody* body, dFloat timestep, int threadIndex);
    void Newton_SetTransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex);
    void Newton_DestroyBodyCallback(const NewtonBody* body);
    unsigned Newton_WorldRayPrefilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData);






    /// Register Physics library objects.
    void URHO3D_API RegisterPhysicsLibrary(Context* context);
}
