#pragma once

#include "../Scene/Component.h"

class NewtonCollision;
class NewtonMesh;
namespace Urho3D
{


    class NewtonNodePhysicsGlue;
    class UrhoNewtonPhysicsWorld;
    class NewtonRigidBody;
    class NewtonMeshObject;
    class NewtonPhysicsMaterial;
    class Component;
    class Model;

    /// base component for attaching collision shapes to nodes.
    class URHO3D_API NewtonCollisionShape : public Component
    {
        URHO3D_OBJECT(NewtonCollisionShape, Component);
    public:

        friend class UrhoNewtonPhysicsWorld;
        friend class NewtonRigidBody;

        NewtonCollisionShape(Context* context);

        virtual ~NewtonCollisionShape();

        static void RegisterObject(Context* context);

        


        void SetPhysicsMaterial(NewtonPhysicsMaterial* material);


        /// Set the positional offset of the shape in local space to the node.
        void SetPositionOffset(Vector3 position) { position_ = position; MarkDirty(true); }

        /// Set the rotational offset of the shape in local space to the node.
        void SetRotationOffset(Quaternion rotation) { rotation_ = rotation; MarkDirty(true); }

        /// Get the positional offset of the shape in local space to the node.
        Vector3 GetPositionOffset() { return position_;}

        /// Get the rotational offset of the shape in local space to the node.
        Quaternion GetRotationOffset() { return rotation_; }

        /// get local offset matrix.
        Matrix3x4 GetOffsetMatrix()
        {
            return Matrix3x4(position_, rotation_, 1.0f);
        }


        /// Returns the volume of the collision shape (convex only);
        float GetVolume() { return volume_; }

        /// Mark the shape as dirty causing it to be rebuilt by the physics world.
        void MarkDirty(bool dirty = true);
        /// Get the current dirty status.
        bool GetDirty() const { return shapeNeedsRebuilt_; }

        /// Returns the internal newton collision
        NewtonCollision* GetNewtonCollision();


        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:

        /// Physics world.
        WeakPtr<UrhoNewtonPhysicsWorld> physicsWorld_;

        /// Internal Newton Collision
        NewtonCollision* newtonCollision_ = nullptr;

        /// newton Mesh reference
        WeakPtr<NewtonMeshObject> newtonMesh_ = nullptr;

        ///reference to physics material resource;
        WeakPtr<NewtonPhysicsMaterial> physicsMaterial_ = nullptr;

        ///optional scene collision node for use if the collision is part of the scene collision of the newton world.
        void* newtonSceneCollisionNode = nullptr;


        /// volume
        float volume_ = 0.0f;
        /// shape dirty flag
        bool shapeNeedsRebuilt_ = true;

        /// Offset position.
        Vector3 position_;
        /// Offset rotation.
        Quaternion rotation_;

        /// updates the intenal newton collision pointer to reference the appropriate collision instance from the newton cache based on current parameters.
        void updateBuild();
        /// implement this in subclasses to create the internal newton collision
        virtual void buildNewtonCollision();
        /// Frees the internal collision shape and mesh;
        void freeInternalCollision();
        void applyMaterial();
        /// Calculates the effective mass based off density and size. (could be expensive)
        float updateVolume();


        void HandleNodeAdded(StringHash event, VariantMap& eventData);
        void HandleNodeRemoved(StringHash event, VariantMap& eventData);
        void HandleNodeTransformChange(StringHash event, VariantMap& eventData);


        virtual void OnNodeSet(Node* node) override;
    };








}
