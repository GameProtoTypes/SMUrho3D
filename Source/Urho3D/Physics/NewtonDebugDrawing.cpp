#include "NewtonDebugDrawing.h"
#include "Newton.h"
#include "Graphics/DebugRenderer.h"
#include "dMatrix.h"
#include "Math/Vector3.h"
#include "NewtonPhysicsWorld.h"
#include "UrhoNewtonConversions.h"


namespace Urho3D {


    void NewtonBodyDebugDrawAABB(NewtonBody* body, DebugRenderer* debug, bool depthTest /*= false*/)
    {
        dMatrix matrix;
        dVector p0(0.0f);
        dVector p1(0.0f);

        NewtonCollision* const collision = NewtonBodyGetCollision(body);
        NewtonBodyGetMatrix(body, &matrix[0][0]);
        NewtonCollisionCalculateAABB(collision, &matrix[0][0], &p0[0], &p1[0]);


        Vector3 min = NewtonToUrhoVec3(p0);
        Vector3 max = NewtonToUrhoVec3(p1);
        BoundingBox box(min, max);
        debug->AddBoundingBox(box, Color::YELLOW, depthTest, false);

        debug->AddSphere(Sphere(Vector3(0,0,0), 5.0f), Color::YELLOW, depthTest);

    }
}
