#pragma once
#include "Resource/Resource.h"
#include "Core/Context.h"
#include "Core/Object.h"
#include "Newton.h"
#include "Math/Interpolations.h"

namespace Urho3D {

    ///physics material resource providing data for a physics surface.
    class URHO3D_API NewtonPhysicsMaterial : public Resource
    {
        URHO3D_OBJECT(NewtonPhysicsMaterial, Resource);

    public:
        friend class NewtonPhysicsMaterialContactPair;
        NewtonPhysicsMaterial(Context* context) : Resource(context)
        {

        }
        virtual ~NewtonPhysicsMaterial() {}

        static void RegisterObject(Context* context) {
            context->RegisterFactory< NewtonPhysicsMaterial>();
        }


    protected:

        //friction, restitution, etc..
        float softness_;//
        float elasticity_;//how "rubbery the surface along the normal
        float slipperyness_;//[0 - 1] where 1 is icy and 0 is glue.

        float staticRugosity_; //roughness used for approximating coefficients of friction. Rugosity = (Surface Area Actual)/(Surface Area Geometrical) [ 1 - 1.1 ]
        float kineticRugosity_;

        float normalAcceleration_;//acceleration to apply along the normal of the surface. 
        Vector2 tangentalAcceleration_; //acceleration to apply along the surface (x,y)
    };


    class URHO3D_API NewtonPhysicsMaterialContactPair : public Object
    {
        URHO3D_OBJECT(NewtonPhysicsMaterialContactPair, Object);
    public:
        NewtonPhysicsMaterialContactPair(Context* context) : Object(context)
        {

        }
        virtual ~NewtonPhysicsMaterialContactPair() {}

        static void RegisterObject(Context* context) {
            context->RegisterFactory< NewtonPhysicsMaterialContactPair>();
        }

        ///computes the metrics of this material contact pair for the given two material definitions.
        void SetMaterials(NewtonPhysicsMaterial* material1, NewtonPhysicsMaterial* material2)
        {
            float reverseSlipperyness1 = 1 - material1->slipperyness_;
            float reverseSlipperyness2 = 1 - material2->slipperyness_;

            float maxStaticCoeff = 2.0f;

            const float maxRugosity = 1.1f;
            staticFrictionCoef_ = BilinearInterpolation(Vector3(1, maxRugosity, reverseSlipperyness2),
                Vector3(maxRugosity, maxRugosity, maxStaticCoeff),
                Vector3(1, 1, Min(reverseSlipperyness1, reverseSlipperyness2)),
                Vector3(maxRugosity, 1, reverseSlipperyness1),
                Vector2(material1->staticRugosity_, material2->staticRugosity_));

            kineticFrictionCoef_ = BilinearInterpolation(Vector3(1, maxRugosity, reverseSlipperyness2),
                Vector3(maxRugosity, maxRugosity, maxStaticCoeff),
                Vector3(1, 1, Min(reverseSlipperyness1, reverseSlipperyness2)),
                Vector3(maxRugosity, 1, reverseSlipperyness1),
                Vector2(material1->staticRugosity_, material2->kineticRugosity_));

        }

    protected:


        float staticFrictionCoef_;
        float kineticFrictionCoef_;

        int newtonMaterialPairGroupId = 0;
    };
}

