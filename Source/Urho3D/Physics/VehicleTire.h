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

        friend class PhysicsVehicle;

        VehicleTire(Context* context);
        ~VehicleTire();

        static void RegisterObject(Context* context);

        ///return the node that will be matched to tire world transform.
        Node* GetNode() { return node_; }

        ///Set mass density
        void SetMassDensity(float massDensity) { massDensity_ = massDensity; }
        float GetMassDensity() const { return massDensity_; }

        void SetRadius(float radius) { radius_ = radius; }
        float GetRadius() const { return radius_; }


        void SetWidth(float width) { width_ = width; }
        float GetWidth() const { return width_; }


        

    protected:


        float massDensity_ = 10.0f;

        float radius_ = 0.5f;
        float width_ = 0.25;



        Matrix3x4 initialWorldTransform_;


        dVehicleTireInterface::dTireInfo* tireInfo_ = nullptr;

        dVehicleTireInterface* tireInterface_ = nullptr;

        WeakPtr<Node> node_;
    };


}

