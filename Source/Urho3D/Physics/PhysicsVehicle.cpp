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
            physicsWorld_->removeVehicle(this);
            physicsWorld_ = nullptr;

            if (vehicleChassis_) {
                delete vehicleChassis_;
                vehicleChassis_ = nullptr;
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


        




        isDirty_ = false;
    }

}
