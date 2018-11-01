#pragma once
#include "../Core/Object.h"
#include "dVehicleInterface.h"


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

        ///return the node that will be moving along with the tire position.
        Node* GetNode() { return node_; }


        Matrix3x4 initialWorldTransform_;


        dVehicleTireInterface::dTireInfo* tireInfo_ = nullptr;

        dVehicleTireInterface* tireInterface_ = nullptr;

        WeakPtr<Node> node_;
    };


}

