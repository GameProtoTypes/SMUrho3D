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
                coneAngle_ = angle;
                MarkDirty();
            }
        }
        float GetConeAngle() const { return coneAngle_; }

        ///Set Twist limits 
        void SetTwistLimits(float minAngle, float maxAngle) {
            if (twistMaxAngle_ != maxAngle || twistMinAngle_ != minAngle) {
                twistMinAngle_ = minAngle;
                twistMaxAngle_ = maxAngle;
                MarkDirty();
            }
        }
        void SetTwistLimitMin(float minAngle) {
            if (twistMinAngle_ != minAngle) {
                twistMinAngle_ = minAngle;
                MarkDirty();
            }
        }
        void SetTwistLimitMax(float maxAngle) {
            if (twistMaxAngle_ != maxAngle) {
                twistMaxAngle_ = maxAngle;
                MarkDirty();
            }
        }

        float GetTwistLimitMin() const { return twistMinAngle_; }
        float GetTwistLimitMax() const { return twistMaxAngle_; }
        Vector2 GetTwistLimits() const { return Vector2(twistMinAngle_, twistMaxAngle_); }

        void SetConeEnabled(bool enabled = true) {
            if (coneEnabled_ != enabled) {
                coneEnabled_ = enabled;
                MarkDirty();
            }
        }
        bool GetConeEnabled() const { return coneEnabled_; }


        void SetTwistLimitsEnabled(bool enabled = false)
        {
            if (twistLimitsEnabled_ != enabled) {
                twistLimitsEnabled_ = enabled;
                MarkDirty();
            }
        }
        bool GetTwistLimitsEnabled() const { return twistLimitsEnabled_; }


        void SetConeFriction(float frictionTorque) {
            if (frictionTorque != coneFriction_) {
                coneFriction_ = frictionTorque;
                MarkDirty();
            }
        }
        float GetConeFriction() const { return coneFriction_; }


        void SetTwistFriction(float frictionTorque) {
            if (twistFriction_ != frictionTorque) {
                twistFriction_ = frictionTorque;
                MarkDirty();
            }
        }
        float GetTwistFriction() const { return twistFriction_; }


        virtual void DrawDebugGeometry(DebugRenderer* debug, bool depthTest) override;

    protected:
        bool coneEnabled_ = true;
        bool twistLimitsEnabled_ = false;
        float coneAngle_ = 20.0f;
        float twistMinAngle_ = -45.0f;
        float twistMaxAngle_ = 45.0f;
        float twistFriction_ = 0.0f;
        float coneFriction_ = 0.0f;

        virtual void buildConstraint() override;
    };


}
