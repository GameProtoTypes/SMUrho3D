
#include "NewtonCollisionShape.h"
#include "NewtonPhysicsWorld.h"
#include "NewtonRigidBody.h"

#include "../Core/Context.h"
#include "../Scene/Component.h"
#include "../Scene/Node.h"
#include "../Scene/Scene.h"
#include "../Graphics/Model.h"
#include "../IO/Log.h"

#include "Newton.h"
#include "dMatrix.h"

#include "UrhoNewtonConversions.h"
#include "Graphics/Geometry.h"
#include "Graphics/VertexBuffer.h"
#include "Graphics/GraphicsDefs.h"
#include "Graphics/IndexBuffer.h"
#include "Graphics/VisualDebugger.h"


namespace Urho3D {



    NewtonCollisionShape::NewtonCollisionShape(Context* context) : Component(context)
    {
    }

    NewtonCollisionShape::~NewtonCollisionShape()
    {
        freeInternalCollision();
    }

    void NewtonCollisionShape::RegisterObject(Context* context)
    {
        bool s = context->RegisterFactory<NewtonCollisionShape>();
    }

    void NewtonCollisionShape::SetBox(const Vector3& size, const Vector3& position, const Quaternion& rotation)
    {
        shapeType_ = SHAPE_BOX;
        size_ = size;
        position_ = position;
        rotation_ = rotation;

        reEvaluateCollision();
        notifyRigidBody();
    }


    void NewtonCollisionShape::SetTriangleMesh(Model* model, unsigned lodLevel /*= 0*/, const Vector3& scale /*= Vector3::ONE*/, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    {
        if (!model)
            return;

        shapeType_ = SHAPE_TRIANGLEMESH;
        size_ = scale;
        position_ = position;
        rotation_ = rotation;
        model_ = model;
        modelLodLevel_ = lodLevel;











        reEvaluateCollision();
        notifyRigidBody();


    }

    void NewtonCollisionShape::SetConvexHull(Model* model, unsigned lodLevel /*= 0*/, const Vector3& scale /*= Vector3::ONE*/, const Vector3& position /*= Vector3::ZERO*/, const Quaternion& rotation /*= Quaternion::IDENTITY*/)
    {
        if (!model)
            return;

        shapeType_ = SHAPE_CONVEXHULL;
        size_ = scale;
        position_ = position;
        rotation_ = rotation;
        model_ = model;
        modelLodLevel_ = lodLevel;



        reEvaluateCollision();
        notifyRigidBody();

    }

    void NewtonCollisionShape::reEvaluateCollision()
{
        // first free any reference to an existing collision.
        freeInternalCollision();

        NewtonWorld* world = GetScene()->GetComponent<NewtonPhysicsWorld>()->GetNewtonWorld();

        Matrix4 mat;
        //mat.SetScale(size_);
        mat.SetTranslation(position_);
        mat.SetRotation(rotation_.RotationMatrix());
        dMatrix nMat = UrhoToNewton(mat);


        if (shapeType_ == SHAPE_BOX) {

            // get a newton collision object (note: the same NewtonCollision could be shared between multiple component so this is not nessecarily unique)
            newtonCollision_ = NewtonCreateBox(world, size_.x_*node_->GetScale().x_,
                size_.y_*node_->GetScale().y_,
                size_.z_*node_->GetScale().z_, 0, &nMat[0][0]);

        }
        ///.....
        else if (shapeType_ == SHAPE_TRIANGLEMESH)
        {
            dVector minBox = UrhoToNewton(model_->GetBoundingBox().min_);
            dVector maxBox = UrhoToNewton(model_->GetBoundingBox().max_);

            newtonCollision_ = NewtonCreateTreeCollision(world, 0);



            const unsigned char* vertexData;
            const unsigned char* indexData;
            unsigned elementSize, indexSize;
            const PODVector<VertexElement>* elements;
            Geometry* geo = model_->GetGeometry(modelGeomIndx_, modelLodLevel_);
            geo->GetRawData(vertexData, elementSize, indexData, indexSize, elements);

            bool hasPosition = VertexBuffer::HasElement(*elements, TYPE_VECTOR3, SEM_POSITION);

            if (vertexData && indexData && hasPosition) {

                unsigned vertexStart = geo->GetVertexStart();
                unsigned vertexCount = geo->GetVertexCount();
                unsigned indexStart = geo->GetIndexStart();
                unsigned indexCount = geo->GetIndexCount();
               
                unsigned positionOffset = VertexBuffer::GetElementOffset(*elements, TYPE_VECTOR3, SEM_POSITION);




                NewtonTreeCollisionBeginBuild(newtonCollision_);

                int faceAddCount = 0;


                //loop through all vertexes
                for (int t = vertexStart; t < vertexStart + vertexCount; t++) {
                    const Vector3& v1 = *reinterpret_cast<const Vector3*>(vertexData + t * elementSize + positionOffset);
                    //GSS<VisualDebugger>()->AddCross(v1, 0.01f, Color::BLUE, false);
                }




                for (unsigned curIdx = indexStart; curIdx < indexStart + indexCount; curIdx += 3)
                {
                    //get indexes
                    unsigned char i1 = *(indexData + curIdx * indexSize);
                    unsigned char i2 = *(indexData + (curIdx + 1) * indexSize);
                    unsigned char i3 = *(indexData + (curIdx + 2) * indexSize);


                    //lookup triangle using indexes.
                    const Vector3& v1 = *reinterpret_cast<const Vector3*>(vertexData + i1 * elementSize + positionOffset);
                    const Vector3& v2 = *reinterpret_cast<const Vector3*>(vertexData + i2 * elementSize + positionOffset);
                    const Vector3& v3 = *reinterpret_cast<const Vector3*>(vertexData + i3 * elementSize + positionOffset);

                    //GSS<VisualDebugger>()->AddTriangle(v1*10.0f, v2*10.0f, v3*10.0f, Color::YELLOW, false);

                    GSS<VisualDebugger>()->AddCross(v1, 0.01f, Color::RED, false);
                    GSS<VisualDebugger>()->AddCross(v2, 0.01f, Color::GREEN, false);
                    GSS<VisualDebugger>()->AddCross(v3, 0.01f, Color::BLUE, false);
                    

                    //copy data.
                    float faceData[9];
                    faceData[0] = v1.x_;
                    faceData[1] = v1.y_;
                    faceData[2] = v1.z_;
                    faceData[3] = v2.x_;
                    faceData[4] = v2.y_;
                    faceData[5] = v2.z_;
                    faceData[6] = v3.x_;
                    faceData[7] = v3.y_;
                    faceData[8] = v3.z_;


                    NewtonTreeCollisionAddFace(newtonCollision_, 3, faceData, sizeof(float) * 3, 0);
                    faceAddCount++;
                }


                NewtonTreeCollisionEndBuild(newtonCollision_, 0);
                URHO3D_LOGINFO(String(faceAddCount) + " Faces added to collision");
            }
        }
    }

    void NewtonCollisionShape::freeInternalCollision()
    {
        if (newtonCollision_) {
            NewtonDestroyCollision(newtonCollision_);//decrement the reference count of the collision.
            newtonCollision_ = nullptr;
        }
    }

    void NewtonCollisionShape::notifyRigidBody()
    {
        rigidBody_ = node_->GetComponent<NewtonRigidBody>();
        if (rigidBody_) {
            rigidBody_->reEvaluateBody();
        }
    }

    NewtonCollision* NewtonCollisionShape::GetNewtonCollision()
    {
        return newtonCollision_;
    }




    void NewtonCollisionShape::updateReferenceToRigidBody()
{
        rigidBody_ = node_->GetComponent<NewtonRigidBody>();
    }

    void NewtonCollisionShape::OnNodeSet(Node* node)
    {
       
    }

    void NewtonCollisionShape::OnSceneSet(Scene* scene)
    {
        if (scene)
        {
            if (scene == node_)
                URHO3D_LOGWARNING(GetTypeName() + " should not be created to the root scene node");

            physicsWorld_ = WeakPtr<NewtonPhysicsWorld>(scene->GetOrCreateComponent<NewtonPhysicsWorld>());

            reEvaluateCollision();


            physicsWorld_->addCollisionShape(this);


            notifyRigidBody();
        }
        else
        {
            freeInternalCollision();

            if (physicsWorld_)
                physicsWorld_->removeCollisionShape(this);


            notifyRigidBody();
        }
    }

    void NewtonCollisionShape::OnMarkedDirty(Node* node)
    {
       // throw std::logic_error("The method or operation is not implemented.");
    }

}
