#pragma once

#include "../Physics/CollisionShape.h"
#include "../Scene/Component.h"

class NewtonCollision;
class NewtonMesh;
namespace Urho3D
{



    class NewtonPhysicsWorld;
    class NewtonRigidBody;
    class Component;
    class URHO3D_API NewtonCollisionShape : public Component
    {
        URHO3D_OBJECT(NewtonCollisionShape, Component);
    public:

        friend class NewtonPhysicsWorld;
        friend class NewtonRigidBody;

        NewtonCollisionShape(Context* context);

        ~NewtonCollisionShape() override;

        static void RegisterObject(Context* context);



        /// Set as a box.
        void SetBox(const Vector3& size, const Vector3& position = Vector3::ZERO, const Quaternion& rotation = Quaternion::IDENTITY);
        /// Set as a triangle mesh from Model. 
        void SetTriangleMesh(Model* model, unsigned lodLevel = 0, const Vector3& scale = Vector3::ONE, const Vector3& position = Vector3::ZERO,
            const Quaternion& rotation = Quaternion::IDENTITY);
        /// Set as a convex hull from Model.
        void SetConvexHull(Model* model, unsigned lodLevel = 0, const Vector3& scale = Vector3::ONE, const Vector3& position = Vector3::ZERO,
            const Quaternion& rotation = Quaternion::IDENTITY);
        /// Set as a compound collision generated by newton.
        void SetCompound(Model* model, unsigned lodLevel = 0, const Vector3& scale = Vector3::ONE, const Vector3& position = Vector3::ZERO,
            const Quaternion& rotation = Quaternion::IDENTITY);


        /// Returns the internal newton collision
        NewtonCollision* GetNewtonCollision();


    protected:

        /// Physics world.
        WeakPtr<NewtonPhysicsWorld> physicsWorld_;
        /// Rigid body.
        WeakPtr<NewtonRigidBody> rigidBody_;
        /// Internal Newton Collision
        NewtonCollision* newtonCollision_ = nullptr;
        /// Internal Newton Mesh
        NewtonMesh* newtonMesh_ = nullptr;

        /// Collision shape type.
        ShapeType shapeType_ = SHAPE_BOX;
        /// Model reference
        WeakPtr<Model> model_;
        /// lod level
        unsigned modelLodLevel_ = 0;
        /// model geometry index to sue
        unsigned modelGeomIndx_ = 0;
        /// Offset position.
        Vector3 position_;
        /// Offset rotation.
        Quaternion rotation_;
        /// Shape size.
        Vector3 size_ = Vector3(1.0f, 1.0f, 1.0f);

        /// updates the intenal newton collision pointer to reference the appropriate collision instance from the newton cache based on current parameters.
        void reEvaluateCollision();

        /// Frees the internal collision shape and mesh;
        void freeInternalCollision();

        /// notifies the sibling rigid body of updates if it exists
        void notifyRigidBody();

        /// Called when there is a change to the rigid body component;
        void updateReferenceToRigidBody();




        void formTriangleMeshCollision();
        void formConvexHullCollision();
        void formCompoundCollision();

        ///forms newtonMesh_ from model geometry for later use.
        bool formTriangleMesh();







        virtual void OnNodeSet(Node* node) override;


        virtual void OnSceneSet(Scene* scene) override;


        virtual void OnMarkedDirty(Node* node) override;

    };



}
