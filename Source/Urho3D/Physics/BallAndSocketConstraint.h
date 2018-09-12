#pragma once


#include "Constraint.h"


namespace Urho3D {
    class Context;
    class URHO3D_API BallAndSocketConstraint : public Constraint
    {
        URHO3D_OBJECT(BallAndSocketConstraint, Constraint);

    public:

        BallAndSocketConstraint(Context* context);
        ~BallAndSocketConstraint();

        static void RegisterObject(Context* context);

        void SetConeAngle(float angle) {
            if (coneAngle_ != angle) {
                coneAngle_ = angle; MarkDirty();
            }
        }

        ///Set Twist limits 
        void SetTwistLimits(float minAngle, float maxAngle) {
            if (twistMaxAngle_ != maxAngle || twistMinAngle_ != minAngle) {
                twistMinAngle_ = minAngle;
                twistMaxAngle_ = maxAngle;
                MarkDirty();
            }
        }

        void SetConeEnabled(bool enabled = true) {
            if (coneEnabled_ != enabled) {
                coneEnabled_ = enabled;
                MarkDirty();
            }

        }

        void SetTwistLimitsEnabled(bool enabled = false)
        {
            if (twistLimitsEnabled_ != enabled) {
                twistLimitsEnabled_ = enabled;
                MarkDirty();
            }
        }



        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:

        bool coneEnabled_ = true;
        bool twistLimitsEnabled_ = false;
        float coneAngle_ = 20.0f;
        float twistMinAngle_ = -45.0f;
        float twistMaxAngle_ = 45.0f;

        virtual void buildConstraint() override;
    };


}
