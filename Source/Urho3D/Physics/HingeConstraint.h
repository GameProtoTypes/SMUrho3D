#pragma once



#include "Constraint.h"


namespace Urho3D {
    class Context;



    class URHO3D_API HingeConstraint : public Constraint
    {
        URHO3D_OBJECT(HingeConstraint, Constraint);

    public:

        HingeConstraint(Context* context);
        ~HingeConstraint();

        static void RegisterObject(Context* context);

        void SetMinAngle(float minAngle)
        {
            if (minAngle_ != minAngle) {
                minAngle_ = minAngle;
                MarkDirty();
            }
        }
        float GetMinAngle() const { return minAngle_; }

        void SetMaxAngle(float maxAngle)
        {
            if (maxAngle_ != maxAngle) {
                maxAngle_ = maxAngle;
                MarkDirty();
            }
        }
        float GetMaxAngle() const { return maxAngle_; }

        void SetFriction(float friction) {
            if (friction_ != friction) {
                friction_ = friction;
                MarkDirty();
            }
        }
        float GetFriction() const { return friction_; }

        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:



        float friction_ = 0.0f;
        bool  enableLimits_ = true;
        float minAngle_ = -45.0f;
        float maxAngle_ = 45.0f;


        virtual void buildConstraint() override;
    };


}
