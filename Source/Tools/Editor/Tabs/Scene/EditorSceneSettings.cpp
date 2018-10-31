//
// Copyright (c) 2018 Rokas Kupstys
// Copyright (c) 2017 Eugene Kozlov
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

#include <Toolbox/Scene/DebugCameraController.h>
#include "EditorSceneSettings.h"
#include "SceneTab.h"
#include "EditorEvents.h"


namespace Urho3D
{

static ResourceRef defaultRenderPath{XMLFile::GetTypeStatic(), "RenderPaths/Forward.xml"};

EditorSceneSettings::EditorSceneSettings(Context* context)
    : Component(context)
    , editorViewportRenderPath_(defaultRenderPath)
{
}

void EditorSceneSettings::RegisterObject(Context* context)
{
    context->RegisterFactory<EditorSceneSettings>();
    URHO3D_ATTRIBUTE("Viewport RenderPath", ResourceRef, editorViewportRenderPath_, defaultRenderPath, AM_DEFAULT);
    URHO3D_ACCESSOR_ATTRIBUTE("Camera Position", GetCameraPosition, SetCameraPosition, Vector3, Vector3::ZERO, AM_EDIT);
    URHO3D_ACCESSOR_ATTRIBUTE("Camera Orthographic Size", GetCameraOrthoSize, SetCameraOrthoSize, float, 0, AM_EDIT);
    URHO3D_ACCESSOR_ATTRIBUTE("Camera Zoom", GetCameraZoom, SetCameraZoom, float, 0, AM_EDIT);
    URHO3D_ACCESSOR_ATTRIBUTE("Camera View 2D", GetCamera2D, SetCamera2D, bool, false, AM_FILE | AM_NOEDIT);
}

void EditorSceneSettings::OnSetAttribute(const AttributeInfo& attr, const Variant& src)
{
    Serializable::OnSetAttribute(attr, src);
    using namespace SceneSettingModified;
    SendEvent(E_SCENESETTINGMODIFIED, P_SCENE, GetScene(), P_NAME, attr.name_, P_VALUE, src);
}

Vector3 EditorSceneSettings::GetCameraPosition() const
{
    if (Node* node = GetCameraNode())
        return node->GetPosition();
    return Vector3::ZERO;
}

void EditorSceneSettings::SetCameraPosition(const Vector3& position)
{
    if (Node* node = GetCameraNode())
    {
        node->SetPosition(position);
        node->LookAt({position.x_, position.y_, 0});
    }
}

float EditorSceneSettings::GetCameraOrthoSize() const
{
    if (Node* node = GetCameraNode())
    {
        if (Camera* camera = node->GetComponent<Camera>())
            return camera->GetOrthoSize();
    }
    return 0;
}

void EditorSceneSettings::SetCameraOrthoSize(float size)
{
    if (Node* node = GetCameraNode())
    {
        if (Camera* camera = node->GetComponent<Camera>())
            camera->SetOrthoSize(size);
    }
}

float EditorSceneSettings::GetCameraZoom() const
{
    if (Node* node = GetCameraNode())
    {
        if (Camera* camera = node->GetComponent<Camera>())
            return camera->GetZoom();
    }
    return 0;
}

void EditorSceneSettings::SetCameraZoom(float zoom)
{
    if (Node* node = GetCameraNode())
    {
        if (Camera* camera = node->GetComponent<Camera>())
            camera->SetZoom(zoom);
    }
}

Node* EditorSceneSettings::GetCameraNode()
{
    return GetScene()->GetChild("__EditorCamera__", true);
}

Node* EditorSceneSettings::GetCameraNode() const
{
    return GetScene()->GetChild("__EditorCamera__", true);
}

void EditorSceneSettings::CreateEditorObjects()
{
    Node* parent = GetScene()->GetChild("EditorObjects");
    if (parent == nullptr)
    {
        parent = GetScene()->CreateChild("EditorObjects", LOCAL, FIRST_INTERNAL_ID);
        parent->AddTag("__EDITOR_OBJECT__");
    }

    Node* camera = parent->GetChild("EditorCamera");
    if (camera == nullptr)
    {
        camera = parent->CreateChild("__EditorCamera__", LOCAL, FIRST_INTERNAL_ID + 1);
        camera->AddTag("__EDITOR_OBJECT__");
    }

    Camera* cameraComponent = camera->GetOrCreateComponent<Camera>();
    cameraComponent->SetFarClip(160000);

    auto* debug = GetScene()->GetOrCreateComponent<DebugRenderer>(LOCAL, FIRST_INTERNAL_ID + 2);
    debug->SetView(cameraComponent);
    debug->SetTemporary(true);
    debug->SetLineAntiAlias(true);

    SetCamera2D(is2D_);
}

bool EditorSceneSettings::GetCamera2D() const
{
    return is2D_;
}

void EditorSceneSettings::SetCamera2D(bool is2D)
{
    // Editor objects may not exist during deserialization. CreateEditorObjects() will call this
    // method again.
    if (Node* camera = GetCameraNode())
    {
        if (is2D)
        {
            camera->RemoveComponent<DebugCameraController>();
            camera->GetOrCreateComponent<DebugCameraController2D>();
            Vector3 pos = camera->GetWorldPosition();
            pos.z_ = Abs(pos.z_);
            camera->SetWorldPosition(pos);
            camera->LookAt(pos + Vector3::FORWARD);
            camera->GetComponent<Camera>()->SetOrthographic(true);
        }
        else
        {
            camera->RemoveComponent<DebugCameraController2D>();
            camera->GetOrCreateComponent<DebugCameraController>();
            camera->GetComponent<Camera>()->SetOrthographic(false);
        }
    }

    is2D_ = is2D;
}

}
