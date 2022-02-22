//
// Copyright (c) 2021-2022 the rbfx project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#pragma once

#include "../TemplateNode.h"
#include "../ParticleGraphNode.h"
#include "../ParticleGraphNodeInstance.h"

namespace Urho3D
{
class ParticleGraphSystem;

namespace ParticleGraphNodes
{
class ConeInstance;

class URHO3D_API Cone : public TemplateNode<ConeInstance, Vector3, Vector3>
{
    URHO3D_OBJECT(Cone, ParticleGraphNode)
public:
    /// Construct Cone.
    explicit Cone(Context* context);
    /// Register particle node factory.
    static void RegisterObject(ParticleGraphSystem* context);

    /// Evaluate size required to place new node instance.
    unsigned EvaluateInstanceSize() const override;

    /// Place new instance at the provided address.
    ParticleGraphNodeInstance* CreateInstanceAt(void* ptr, ParticleGraphLayerInstance* layer) override;

    /// Set Radius.
    void SetRadius(float value);
    /// Get Radius.
    float GetRadius() const;

    /// Set Radius Thickness.
    void SetRadiusThickness(float value);
    /// Get Radius Thickness.
    float GetRadiusThickness() const;

    /// Set Angle.
    void SetAngle(float value);
    /// Get Angle.
    float GetAngle() const;

    /// Set Length.
    void SetLength(float value);
    /// Get Length.
    float GetLength() const;

    /// Set Translation.
    void SetTranslation(Vector3 value);
    /// Get Translation.
    Vector3 GetTranslation() const;

    /// Set Rotation.
    void SetRotation(Quaternion value);
    /// Get Rotation.
    Quaternion GetRotation() const;

    /// Set Scale.
    void SetScale(Vector3 value);
    /// Get Scale.
    Vector3 GetScale() const;

    /// Set From.
    void SetFrom(int value);
    /// Get From.
    int GetFrom() const;

protected:
    float radius_{};
    float radiusThickness_{};
    float angle_{};
    float length_{};
    Vector3 translation_{};
    Quaternion rotation_{};
    Vector3 scale_{};
    int from_{};
};

} // namespace ParticleGraphNodes

} // namespace Urho3D
