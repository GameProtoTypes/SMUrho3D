#include "VehicleTire.h"
#include "Core/Object.h"
#include "Core/Context.h"
#include "dVehicleInterface.h"
#include "PhysicsWorld.h"
#include "Graphics/Model.h"
#include "Scene/Node.h"


namespace Urho3D {


    VehicleTire::VehicleTire(Context* context) : Object(context)
    {
        tireInfo_ = new dVehicleTireInterface::dTireInfo();
        tireInfo_->m_mass = 40.0f;
        tireInfo_->m_radio = 0.5f;
        tireInfo_->m_width = 0.25f;

        tireInfo_->m_pivotOffset = 0.01f;
        tireInfo_->m_steerRate = 0.5f * dPi;
        tireInfo_->m_frictionCoefficient = 0.8f;
        tireInfo_->m_maxSteeringAngle = 20.0f * dDegreeToRad;

        tireInfo_->m_suspensionLength = 0.22f;
        tireInfo_->m_dampingRatio = 15.0f * 100.0f/*vehicleMass*/;
        tireInfo_->m_springStiffness = dAbs(100.0f/*vehicleMass*/ * -9.81f * 8.0f / tireInfo_->m_suspensionLength);

        tireInfo_->m_corneringStiffness = dAbs(100.0f/*vehicleMass*/ * -9.81f * 1.0f);
        tireInfo_->m_longitudinalStiffness = dAbs(100.0f/*vehicleMass*/ * -9.81f * 1.0f);

    }

    VehicleTire::~VehicleTire()
    {
        delete tireInfo_;
    }

    void VehicleTire::RegisterObject(Context* context)
    {
        context->RegisterFactory<VehicleTire>(DEF_PHYSICS_CATEGORY.CString());
    }

    void VehicleTire::SetModel(Model* model)
    {
        model_ = model;
    }

    void VehicleTire::SetVisualRotationOffset(Quaternion rotation)
    {
        visualWorldRotation_ = rotation;
    }

}
