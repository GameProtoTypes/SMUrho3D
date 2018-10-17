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
    }

    VehicleTire* PhysicsVehicle::AddTire(Matrix3x4 worldTransform)
    {

        SharedPtr<VehicleTire> tire = context_->CreateObject<VehicleTire>();
        tire->initialWorldTransform_ = worldTransform;
        tires_ += tire;

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

    void PhysicsVehicle::updateBuild()
    {




        rigidBody_ = node_->GetOrCreateComponent<RigidBody>();

        if (!rigidBody_->GetNewtonBody())
            return;


        Matrix3x4 worldTransform = physicsWorld_->SceneToPhysics_Domain(node_->GetWorldTransform());

        

        NewtonApplyForceAndTorque callback = NewtonBodyGetForceAndTorqueCallback(rigidBody_->GetNewtonBody());//use existing callback.
        vehicleChassis_ = physicsWorld_->vehicleManager_->CreateSingleBodyVehicle(rigidBody_->GetNewtonBody(), UrhoToNewton(Matrix3x4(worldTransform.Translation(), worldTransform.Rotation(), 1.0f)), callback, 1.0f);


        int i = 0;
        for (VehicleTire* tire : tires_)
        {

            tire->node_ = node_->CreateChild("Tire" + String(i));
            tire->node_->SetWorldTransform(tire->initialWorldTransform_.Translation(), tire->initialWorldTransform_.Rotation());
            StaticModel* stmdl = tire->node_->CreateComponent<StaticModel>();
            stmdl->SetModel(tire->model_);



            tire->tireInterface_ = vehicleChassis_->AddTire(UrhoToNewton(tire->initialWorldTransform_), *tire->tireInfo_);

            i++;
        }



        vehicleChassis_->Finalize();


        isDirty_ = false;
    }

    void PhysicsVehicle::applyTransforms()
    {
        if (isDirty_)
            return;

        for (VehicleTire* tire : tires_)
        {
            Matrix3x4 worldTransform = physicsWorld_->PhysicsToScene_Domain(Matrix3x4(NewtonToUrhoMat4(tire->tireInterface_->GetGlobalMatrix())));
            node_->SetWorldTransform(worldTransform.Translation(), worldTransform.Rotation());
        }
    }


}
