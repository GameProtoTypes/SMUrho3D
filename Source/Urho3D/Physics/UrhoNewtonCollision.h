#pragma once

#include "dNewtonCollision.h"


class UrhoNewtonCollision : public dNewtonCollision
{

public:
    UrhoNewtonCollision(dCollsionType type, dLong collisionMask) : dNewtonCollision(type, collisionMask) {}
    virtual ~UrhoNewtonCollision() {};


    virtual dNewtonCollision* Clone(NewtonCollision* const shape) const override
    {
        return nullptr;
    }

    virtual void DebugRender(const dFloat* const matrix, dDebugRenderer* const renderer) const override
    {
    }


    void SetShape(NewtonCollision* const shape)
    {
        dNewtonCollision::SetShape(shape);
    }






};
