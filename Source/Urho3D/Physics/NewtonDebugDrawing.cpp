#include "NewtonDebugDrawing.h"
#include "Newton.h"
#include "Graphics/DebugRenderer.h"
#include "dMatrix.h"
#include "Math/Vector3.h"
#include "UrhoNewtonPhysicsWorld.h"
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

    void NewtonBodyDebugDrawCenterOfMass(NewtonBody* body, DebugRenderer* debug, bool depthTest /*= false*/)
    {
            dMatrix matrix;
            dVector com(0.0f);
            dVector p0(0.0f);
            dVector p1(0.0f);


            NewtonCollision* const collision = NewtonBodyGetCollision(body);
            NewtonBodyGetCentreOfMass(body, &com[0]);
            NewtonBodyGetMatrix(body, &matrix[0][0]);
            NewtonCollisionCalculateAABB(collision, &matrix[0][0], &p0[0], &p1[0]);

            Vector3 aabbMin = NewtonToUrhoVec3(p0);
            Vector3 aabbMax = NewtonToUrhoVec3(p1);
            float aabbSize = (aabbMax - aabbMin).Length()*0.1f;

            dVector o(matrix.TransformVector(com));

            dVector x(o + matrix.RotateVector(dVector(1.0f, 0.0f, 0.0f, 0.0f))*aabbSize);
            debug->AddLine(Vector3((o.m_x), (o.m_y), (o.m_z)), Vector3((x.m_x), (x.m_y), (x.m_z)), Color::RED, depthTest);


            dVector y(o + matrix.RotateVector(dVector(0.0f, 1.0f, 0.0f, 0.0f))*aabbSize);
            debug->AddLine(Vector3((o.m_x), (o.m_y), (o.m_z)), Vector3((y.m_x), (y.m_y), (y.m_z)), Color::GREEN, depthTest);


            dVector z(o + matrix.RotateVector(dVector(0.0f, 0.0f, 1.0f, 0.0f))*aabbSize);
            debug->AddLine(Vector3((o.m_x), (o.m_y), (o.m_z)), Vector3((z.m_x), (z.m_y), (z.m_z)), Color::BLUE, depthTest);
    }

    void DebugShowGeometryCollision(void* userData, int vertexCount, const dFloat* const faceVertec, int id)
    {
        debugRenderOptions* options = static_cast<debugRenderOptions*>(userData);


        //if (mode == m_lines) {
        int index = vertexCount - 1;
        dVector p0(faceVertec[index * 3 + 0], faceVertec[index * 3 + 1], faceVertec[index * 3 + 2]);
        for (int i = 0; i < vertexCount; i++) {
            dVector p1(faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
            // glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
            // glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));

            options->debug->AddLine(Vector3((p0.m_x), (p0.m_y), (p0.m_z)), Vector3((p1.m_x), (p1.m_y), (p1.m_z)), options->color, options->depthTest);

            p0 = p1;
        }
        //}
        //else {
        //    dVector p0(faceVertec[0 * 3 + 0], faceVertec[0 * 3 + 1], faceVertec[0 * 3 + 2]);
        //    dVector p1(faceVertec[1 * 3 + 0], faceVertec[1 * 3 + 1], faceVertec[1 * 3 + 2]);
        //    dVector p2(faceVertec[2 * 3 + 0], faceVertec[2 * 3 + 1], faceVertec[2 * 3 + 2]);

        //    dVector normal((p1 - p0).CrossProduct(p2 - p0));
        //    normal = normal.Scale(1.0f / dSqrt(normal.DotProduct3(normal)));
        //    glNormal3f(GLfloat(normal.m_x), GLfloat(normal.m_y), GLfloat(normal.m_z));
        //    for (int i = 2; i < vertexCount; i++) {
        //        p2 = dVector(faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
        //        glVertex3f(GLfloat(p0.m_x), GLfloat(p0.m_y), GLfloat(p0.m_z));
        //        glVertex3f(GLfloat(p1.m_x), GLfloat(p1.m_y), GLfloat(p1.m_z));
        //        glVertex3f(GLfloat(p2.m_x), GLfloat(p2.m_y), GLfloat(p2.m_z));
        //        p1 = p2;
        //    }
        //}
    }

    void NewtonBodyDebugShowCollision(const NewtonBody* const body, DebugRenderer* debug, bool depthTest /*= false*/)
    {
        debugRenderOptions options;
        options.debug = debug;
        options.depthTest = depthTest;


        switch (NewtonBodyGetType(body))
        {
        case NEWTON_DYNAMIC_BODY:
        {
            int sleepState = NewtonBodyGetSleepState(body);
            if (sleepState == 1) {
                // indicate when body is sleeping
                options.color = Color::BLUE;
            }
            else {
                // body is active
                options.color = Color::RED;
            }
            break;
        }

        case NEWTON_KINEMATIC_BODY:
            options.color = Color::WHITE;
            break;
        }
        dMatrix matrix;
        NewtonBodyGetMatrix(body, &matrix[0][0]);

        NewtonCollisionForEachPolygonDo(NewtonBodyGetCollision(body), &matrix[0][0], DebugShowGeometryCollision, (void*)&options);
    }

    void NewtonBodyDebugContactForces(const NewtonBody* const body, float scaleFactor, DebugRenderer* debug, bool depthTest /*= false*/)
    {
        dFloat mass;
        dFloat Ixx;
        dFloat Iyy;
        dFloat Izz;
        NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

        //draw normal forces in term of acceleration.
        //this mean that two bodies with same shape but different mass will display the same force
        if (mass > 0.0f) {
            scaleFactor = scaleFactor / mass;
            for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body); joint; joint = NewtonBodyGetNextContactJoint(body, joint)) {
                if (NewtonJointIsActive(joint)) {
                    for (void* contact = NewtonContactJointGetFirstContact(joint); contact; contact = NewtonContactJointGetNextContact(joint, contact)) {
                        dVector point(0.0f);
                        dVector normal(0.0f);
                        dVector tangnetDir0(0.0f);
                        dVector tangnetDir1(0.0f);
                        dVector contactForce(0.0f);
                        NewtonMaterial* const material = NewtonContactGetMaterial(contact);

                        NewtonMaterialGetContactForce(material, body, &contactForce.m_x);
                        NewtonMaterialGetContactPositionAndNormal(material, body, &point.m_x, &normal.m_x);
                        dVector normalforce(normal.Scale(contactForce.DotProduct3(normal)));
                        dVector p0(point);
                        dVector p1(point + normalforce.Scale(scaleFactor));

                        debug->AddLine(Vector3((p0.m_x), (p0.m_y), (p0.m_z)), Vector3((p1.m_x), (p1.m_y), (p1.m_z)), Color::BLUE, depthTest);



                        // these are the components of the tangents forces at the contact point, the can be display at the contact position point.
                        NewtonMaterialGetContactTangentDirections(material, body, &tangnetDir0[0], &tangnetDir1[0]);
                        dVector tangentForce1(tangnetDir0.Scale((contactForce.DotProduct3(tangnetDir0)) * scaleFactor));
                        dVector tangentForce2(tangnetDir1.Scale((contactForce.DotProduct3(tangnetDir1)) * scaleFactor));

                        p1 = point + tangentForce1.Scale(scaleFactor);
                        debug->AddLine(Vector3((p0.m_x), (p0.m_y), (p0.m_z)), Vector3((p1.m_x), (p1.m_y), (p1.m_z)), Color::BLUE, depthTest);


                        p1 = point + tangentForce2.Scale(scaleFactor);
                        debug->AddLine(Vector3((p0.m_x), (p0.m_y), (p0.m_z)), Vector3((p1.m_x), (p1.m_y), (p1.m_z)), Color::BLUE, depthTest);
                    }
                }
            }
        }
    }

    void NewtonDebugShowJoints(NewtonWorld* newtonWorld, DebugRenderer* debug, bool depthTest /*= false*/)
    {
        //// this will go over the joint list twice, 
        //for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
        //    for (NewtonJoint* joint = NewtonBodyGetFirstJoint(body); joint; joint = NewtonBodyGetNextJoint(body, joint)) {
        //        dCustomJoint* const customJoint = (dCustomJoint*)NewtonJointGetUserData(joint);
        //        if (customJoint) {
        //            customJoint->Debug(jointDebug);
        //        }
        //    }
        //}
        //NewtonWorldListenerDebug(world, jointDebug);
    }

    void NewtonCollisionDraw(NewtonCollision* collision, const Matrix3x4& transform, DebugRenderer* debug, bool depthTest /*= false*/)
    {
        debugRenderOptions options;
        options.debug = debug;
        options.depthTest = depthTest;

        NewtonCollisionForEachPolygonDo(collision, &UrhoToNewton(transform)[0][0], DebugShowGeometryCollision, (void*)&options);
    }

}
