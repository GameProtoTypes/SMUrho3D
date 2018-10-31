#include "PhysicsVehicle.h"
#include "Core/Context.h"
#include "PhysicsWorld.h"
#include "Scene/Component.h"
#include "Scene/Scene.h"
#include "CollisionShapesDerived.h"
#include "../dVehicle/dVehicleManager.h"
#include "../dVehicle/dVehicleChassis.h"
#include "RigidBody.h"
#include "UrhoNewtonConversions.h"
#include "Container/Ptr.h"
#include "Graphics/Model.h"
#include "Graphics/StaticModel.h"
#include "VehicleTire.h"
#include "NewtonDebugDrawing.h"

namespace Urho3D {





    PhysicsVehicle::PhysicsVehicle(Context* context) : Component(context)
    {

    }

    PhysicsVehicle::~PhysicsVehicle()
    {

    }

    void PhysicsVehicle::RegisterObject(Context* context)
    {
        context->RegisterFactory<PhysicsVehicle>(DEF_PHYSICS_CATEGORY.CString());
    }

    void PhysicsVehicle::DrawDebugGeometry(DebugRenderer* debug, bool depthTest)
    {
        Component::DrawDebugGeometry(debug, depthTest);

        //NewtonDebug_BodyDrawCollision(physicsWorld_, tires_[0]->tireInterface_->GetCollisionShape(), debug, depthTest);


        for (VehicleTire* tire : tires_)
        {

            dMatrix matrix = tire->tireInterface_->GetGlobalMatrix();

            Matrix3x4 mat = Matrix3x4(NewtonToUrhoMat4(matrix));




            debugRenderOptions options;
            options.debug = debug;
            options.depthTest = depthTest;



            matrix = UrhoToNewton(mat);
            NewtonCollisionForEachPolygonDo(tire->tireInterface_->GetCollisionShape(), &matrix[0][0], NewtonDebug_ShowGeometryCollisionCallback, (void*)&options);

        }
    }

    VehicleTire* PhysicsVehicle::AddTire(Matrix3x4 worldTransform)
    {

        SharedPtr<VehicleTire> tire = context_->CreateObject<VehicleTire>();
        tire->initialWorldTransform_ = worldTransform;
        tires_ += tire;

        MarkDirty();

        return tire;
    }

    void PhysicsVehicle::OnNodeSet(Node* node)
    {
        if (node)
        {

            ///auto create physics world
            physicsWorld_ = WeakPtr<PhysicsWorld>(GetScene()->GetOrCreateComponent<PhysicsWorld>());

            physicsWorld_->addVehicle(this);
            

        }
        else
        {
            if (physicsWorld_) {
                physicsWorld_->removeVehicle(this);
            }

        }
    }

    void PhysicsVehicle::reBuild()
    {
        if (vehicleChassis_)
        {
            physicsWorld_->vehicleManager_->DestroyController(vehicleChassis_);
            //NewtonDestroyBody(internalBody_);
            //internalBody_ = nullptr;
            vehicleChassis_ = nullptr;
        }



        rigidBody_ = node_->GetOrCreateComponent<RigidBody>();
        NewtonBody* body = rigidBody_->GetNewtonBody();
        //internalBody_ = body;
        if (!body)
            return;


        Matrix3x4 worldTransform = physicsWorld_->SceneToPhysics_Domain(node_->GetWorldTransform());

        

        NewtonApplyForceAndTorque callback = NewtonBodyGetForceAndTorqueCallback(rigidBody_->GetNewtonBody());//use existing callback.
        vehicleChassis_ = physicsWorld_->vehicleManager_->CreateSingleBodyVehicle(body, UrhoToNewton(Matrix3x4(worldTransform.Translation(), worldTransform.Rotation(), 1.0f)), callback, 1.0f);


        int i = 0;
        for (VehicleTire* tire : tires_)
        {

            tire->node_ = node_->CreateChild("Tire" + String(i));
            tire->node_->SetWorldTransform(tire->initialWorldTransform_.Translation(), tire->visualWorldRotation_);
            StaticModel* stmdl = tire->node_->CreateComponent<StaticModel>();
            stmdl->SetModel(tire->model_);



            tire->tireInterface_ = vehicleChassis_->AddTire(UrhoToNewton(tire->initialWorldTransform_), *tire->tireInfo_);
            
            i++;
        }



        vehicleChassis_->Finalize();


        RigidBody* backTest = (RigidBody*)NewtonBodyGetUserData(body);



        isDirty_ = false;
    }

    void PhysicsVehicle::applyTransforms()
    {
        if (isDirty_)
            return;

        for (VehicleTire* tire : tires_)
        {
            Matrix3x4 worldTransform = physicsWorld_->PhysicsToScene_Domain(Matrix3x4(NewtonToUrhoMat4(tire->tireInterface_->GetGlobalMatrix())));
            tire->node_->SetWorldTransform(worldTransform.Translation(), worldTransform.Rotation() * tire->visualWorldRotation_);
        }
    }


}
