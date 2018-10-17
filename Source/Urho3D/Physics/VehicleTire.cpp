#include "VehicleTire.h"
#include "Core/Object.h"
#include "Core/Context.h"
#include "dVehicleTireInterface.h"
#include "PhysicsWorld.h"
#include "Graphics/Model.h"
#include "Scene/Node.h"


namespace Urho3D {


    VehicleTire::VehicleTire(Context* context) : Object(context)
    {
        tireInfo_ = new dVehicleTireInterface::dTireInfo();
        tireInfo_->m_mass = 1.0f;
        tireInfo_->m_radio = 0.5f;
        tireInfo_->m_width = 0.2f;

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

}
