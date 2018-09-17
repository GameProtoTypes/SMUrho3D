#pragma once

#include "../Scene/Component.h"

class NewtonCollision;
class NewtonMesh;
namespace Urho3D
{


    class NewtonNodePhysicsGlue;
    class PhysicsWorld;
    class RigidBody;
    class NewtonMeshObject;
    class PhysicsMaterial;
    class Component;
    class Model;

    /// base component for attaching collision shapes to nodes.
    class URHO3D_API CollisionShape : public Component
    {
        URHO3D_OBJECT(CollisionShape, Component);
    public:

        friend class PhysicsWorld;
        friend class RigidBody;

        CollisionShape(Context* context);

        virtual ~CollisionShape();

        static void RegisterObject(Context* context);

        


        //void SetPhysicsMaterial(PhysicsMaterial* material);


        /// Set the positional offset of the shape in local space to the node.
        void SetPositionOffset(Vector3 position) { position_ = position; MarkDirty(true); }

        /// Set the scale factor to apply to this shape.
        void SetScaleFactor(Vector3 scale) { scale_ = scale; MarkDirty(true); }

        /// Set the rotational offset of the shape in local space to the node.
        void SetRotationOffset(Quaternion rotation) { rotation_ = rotation; MarkDirty(true); }

        /// Get the positional offset of the shape in local space to the node.
        Vector3 GetPositionOffset() const { return position_;}

        /// Get the scale factor of the shape that is applied on top of node.
        Vector3 GetScaleFactor() const { return scale_; }

        /// Get the rotational offset of the shape in local space to the node.
        Quaternion GetRotationOffset() const { return rotation_; }




        /// get local offset matrix.
        Matrix3x4 GetOffsetMatrix()
        {
            return Matrix3x4(position_, rotation_, 1.0f);
        }

        /// Mark the shape as dirty causing it to be rebuilt by the physics world.
        void MarkDirty(bool dirty = true);
        /// Get the current dirty status.
        bool GetDirty() const { return shapeNeedsRebuilt_; }

        /// Returns the internal newton collision
        NewtonCollision* GetNewtonCollision();

        bool GetDrawNewtonDebugGeometry() {
            return drawPhysicsDebugCollisionGeometry_;
        }

        void SetDrawNewtonDebugGeometry(bool enable) { drawPhysicsDebugCollisionGeometry_ = enable; }

        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;


        virtual void OnSetEnabled() override;

    protected:

        /// Physics world.
        WeakPtr<PhysicsWorld> physicsWorld_;

        /// Internal Newton Collision
        NewtonCollision* newtonCollision_ = nullptr;

        /// newton Mesh reference
        WeakPtr<NewtonMeshObject> newtonMesh_ = nullptr;

        ///reference to physics material resource;
        WeakPtr<PhysicsMaterial> physicsMaterial_ = nullptr;

        ///optional scene collision node for use if the collision is part of the scene collision of the newton world.
        void* newtonSceneCollisionNode = nullptr;

        /// shape dirty flag
        bool shapeNeedsRebuilt_ = true;

        /// Offset position.
        Vector3 position_;
        /// Scale Factor
        Vector3 scale_ = Vector3::ONE;
        /// Offset rotation.
        Quaternion rotation_;

        bool drawPhysicsDebugCollisionGeometry_ = true;

        /// updates the intenal newton collision pointer to reference the appropriate collision instance from the newton cache based on current parameters.
        void updateBuild();
        /// implement this in subclasses to create the internal newton collision
        virtual void buildNewtonCollision();
        /// Frees the internal collision shape and mesh;
        void freeInternalCollision();



        void MarkRigidBodyDirty();

        void HandleNodeAdded(StringHash event, VariantMap& eventData);
        void HandleNodeRemoved(StringHash event, VariantMap& eventData);
        void HandleNodeTransformChange(StringHash event, VariantMap& eventData);


        virtual void OnNodeSet(Node* node) override;

        virtual void OnNodeSetEnabled(Node* node) override;

    };








}
