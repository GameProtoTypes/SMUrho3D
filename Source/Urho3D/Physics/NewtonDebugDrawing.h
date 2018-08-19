#pragma once
#include "Newton.h"
#include "../Math/Color.h"


class NewtonBody;

namespace Urho3D
{
    class DebugRenderer;
    class Matrix3x4;

    struct debugRenderOptions {
        Color color = Color::GRAY;
        DebugRenderer* debug;
        bool depthTest = false;
    };



    void NewtonDebug_BodyDrawAABB(NewtonBody* body, DebugRenderer* debug, bool depthTest = false);
    void NewtonDebug_BodyDrawCenterOfMass(NewtonBody* body, DebugRenderer* debug, bool depthTest = false);
    void NewtonDebug_BodyDrawCollision(const NewtonBody* const body, DebugRenderer* debug, bool depthTest = false);
    void NewtonDebug_BodyDrawContactForces(const NewtonBody* const body, float scaleFactor, DebugRenderer* debug, bool depthTest = false);
    void NewtonDebug_DrawJoints(NewtonWorld* newtonWorld, DebugRenderer* debug, bool depthTest = false);
    void NewtonDebug_DrawCollision(NewtonCollision* collision, const Matrix3x4& transform, const Color& color, DebugRenderer* debug, bool depthTest = false);




    void NewtonDebug_ShowGeometryCollisionCallback(void* userData, int vertexCount, const dFloat* const faceVertec, int id);



}
