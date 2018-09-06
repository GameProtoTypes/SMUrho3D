#include "PhysicsWorld.h"
#include "Math/Sphere.h"
#include "UrhoNewtonConversions.h"
#include "dMatrix.h"
#include "RigidBody.h"

namespace Urho3D {



    void PhysicsWorld::GetRigidBodies(PODVector<RigidBody*>& result, const Sphere& sphere, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {

        Matrix3x4 mat;
        mat.SetTranslation(sphere.center_);

        NewtonCollision* newtonShape = UrhoShapeToNewtonCollision(newtonWorld_, sphere, false);
        int numContacts = DoNewtonCollideTest(&UrhoToNewton(mat)[0][0], newtonShape);

        GetBodiesInConvexCast(result, numContacts);

        NewtonDestroyCollision(newtonShape);
    }

    void PhysicsWorld::GetRigidBodies(PODVector<RigidBody*>& result, const BoundingBox& box, unsigned collisionMask /*= M_MAX_UNSIGNED*/)
    {
        Matrix3x4 mat;
        mat.SetTranslation(box.Center());

        NewtonCollision* newtonShape = UrhoShapeToNewtonCollision(newtonWorld_, box, false);
        int numContacts = DoNewtonCollideTest(&UrhoToNewton(mat)[0][0], newtonShape);

        GetBodiesInConvexCast(result, numContacts);
        NewtonDestroyCollision(newtonShape);

    }


    void PhysicsWorld::GetRigidBodies(PODVector<RigidBody*>& result, const RigidBody* body)
    {
        dMatrix mat;
        NewtonBodyGetMatrix(body->GetNewtonBody(), &mat[0][0]);
        NewtonCollision* newtonShape = body->GetEffectiveNewtonCollision();
        int numContacts = DoNewtonCollideTest(&mat[0][0], newtonShape);

        GetBodiesInConvexCast(result, numContacts);
    }


}

