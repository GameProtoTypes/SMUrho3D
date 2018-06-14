#pragma once


class NewtonBody;

namespace Urho3D
{
    class DebugRenderer;

    void NewtonBodyDebugDrawAABB(NewtonBody* body, DebugRenderer* debug, bool depthTest = false);
}
