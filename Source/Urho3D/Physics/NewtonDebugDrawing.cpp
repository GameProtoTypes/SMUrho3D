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

}
