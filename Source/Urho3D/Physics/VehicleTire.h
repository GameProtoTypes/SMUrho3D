#pragma once
#include "../Core/Object.h"
#include "dVehicleTireInterface.h"


class dVehicleTireInterface;
class dVehicleTireInterface::dTireInfo;

namespace Urho3D {
    class Node;
    class Model;

    class URHO3D_API VehicleTire : public Object {
        URHO3D_OBJECT(VehicleTire, Object);
    public:
        VehicleTire(Context* context);
        ~VehicleTire();

        static void RegisterObject(Context* context);

        void SetModel(Model* model);

        void SetVisualRotationOffset(Quaternion rotation);

        Matrix3x4 initialWorldTransform_;

        Quaternion visualWorldRotation_;

        dVehicleTireInterface::dTireInfo* tireInfo_ = nullptr;

        dVehicleTireInterface* tireInterface_ = nullptr;

        WeakPtr<Model> model_;
        WeakPtr<Node> node_;
    };


}

