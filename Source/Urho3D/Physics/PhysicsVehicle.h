#pragma once
#include "../Scene/Component.h"

class dVehicleChassis;
namespace Urho3D {

    class RigidBody;
    class CollisionShape;
    class PhysicsWorld;

    /// component for creations of specialized physics vehicle.
    class URHO3D_API PhysicsVehicle : public Component
    {
        URHO3D_OBJECT(PhysicsVehicle, Component);
    public:

        friend class PhysicsWorld;

        PhysicsVehicle(Context* context);
        virtual ~PhysicsVehicle();

        /// Register object factory.
        static void RegisterObject(Context* context);

        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest);

        void MarkDirty(bool dirty = true) { isDirty_ = dirty; }









    protected:



        virtual void OnNodeSet(Node* node) override;

        void updateBuild();


        WeakPtr<RigidBody> rigidBody_;

        //WeakPtr<CollisionShape> colShape_;

        WeakPtr<PhysicsWorld> physicsWorld_;

        bool isDirty_ = true;

        dVehicleChassis* vehicleChassis_ = nullptr;

    };




}

