//
// Copyright (c) 2008-2018 the Urho3D project.
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

#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Engine/Engine.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/Graphics/DebugRenderer.h>
#include <Urho3D/Graphics/Graphics.h>
#include <Urho3D/Graphics/Light.h>
#include <Urho3D/Graphics/Material.h>
#include <Urho3D/Graphics/Model.h>
#include <Urho3D/Graphics/Octree.h>
#include <Urho3D/Graphics/Renderer.h>
#include <Urho3D/Graphics/Skybox.h>
#include <Urho3D/Graphics/Zone.h>
#include <Urho3D/Input/Input.h>
#include <Urho3D/IO/File.h>
#include <Urho3D/IO/FileSystem.h>
#include <Urho3D/Physics/NewtonCollisionShape.h>
#include <Urho3D/Physics/NewtonCollisionShapesDerived.h>
#include "Urho3D/Physics/UrhoNewtonPhysicsWorld.h"
#include <Urho3D/Physics/NewtonRigidBody.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/UI/Font.h>
#include <Urho3D/UI/Text.h>
#include <Urho3D/UI/UI.h>
#include "Physics.h"
#include "PhysicsSamplesUtils.h"

#include <Urho3D/DebugNew.h>
#include "Urho3D/Graphics/VisualDebugger.h"
#include "Urho3D/Physics/NewtonFixedDistanceConstraint.h"
#include "Urho3D/Physics/NewtonPhysicsMaterial.h"
#include "Urho3D/Physics/NewtonBallAndSocketConstraint.h"
#include "Urho3D/Physics/NewtonKinematicsJoint.h"



URHO3D_DEFINE_APPLICATION_MAIN(Physics)

Physics::Physics(Context* context) :
    Sample(context),
    drawDebug_(false)
{
}

void Physics::Start()
{

    context_->RegisterSubsystem<VisualDebugger>();


    // Execute base class startup
    Sample::Start();

    // Create the scene content
    CreateScene();

    // Create the UI content
    CreateInstructions();

    // Setup the viewport for displaying the scene
    SetupViewport();

    // Hook up to the frame update and render post-update events
    SubscribeToEvents();

    // Set the mouse mode to use in the sample
    Sample::InitMouseMode(MM_RELATIVE);
}

void Physics::CreateScene()
{
    auto* cache = GetSubsystem<ResourceCache>();

    scene_ = new Scene(context_);

    // Create octree, use default volume (-1000, -1000, -1000) to (1000, 1000, 1000)
    // Create a physics simulation world with default parameters, which will update at 60fps. the Octree must
    // exist before creating drawable components, the PhysicsWorld must exist before creating physics components.
    // Finally, create a DebugRenderer component so that we can draw physics debug geometry
    scene_->CreateComponent<Octree>();
    UrhoNewtonPhysicsWorld* newtonWorld = scene_->CreateComponent<UrhoNewtonPhysicsWorld>();
    newtonWorld->SetGravity(Vector3(0, -9.81f, 0));
    //scene_->CreateComponent<NewtonCollisionShape_SceneCollision>();
    scene_->CreateComponent<DebugRenderer>();

    // Create a Zone component for ambient lighting & fog control
    Node* zoneNode = scene_->CreateChild("Zone");
    auto* zone = zoneNode->CreateComponent<Zone>();
    zone->SetBoundingBox(BoundingBox(-1000.0f, 1000.0f));
    zone->SetAmbientColor(Color(0.15f, 0.15f, 0.15f));
    zone->SetFogColor(Color(1.0f, 1.0f, 1.0f));
    zone->SetFogStart(300.0f);
    zone->SetFogEnd(500.0f);

    // Create a directional light to the world. Enable cascaded shadows on it
    Node* lightNode = scene_->CreateChild("DirectionalLight");
    lightNode->SetDirection(Vector3(0.6f, -1.0f, 0.8f));
    auto* light = lightNode->CreateComponent<Light>();
    light->SetLightType(LIGHT_DIRECTIONAL);
    light->SetCastShadows(true);
    light->SetShadowBias(BiasParameters(0.00025f, 0.5f));
    // Set cascade splits at 10, 50 and 200 world units, fade shadows out at 80% of maximum shadow distance
    light->SetShadowCascade(CascadeParameters(10.0f, 50.0f, 200.0f, 0.0f, 0.8f));

    // Create skybox. The Skybox component is used like StaticModel, but it will be always located at the camera, giving the
    // illusion of the box planes being far away. Use just the ordinary Box model and a suitable material, whose shader will
    // generate the necessary 3D texture coordinates for cube mapping
    Node* skyNode = scene_->CreateChild("Sky");
    skyNode->SetScale(500.0f); // The scale actually does not matter
    auto* skybox = skyNode->CreateComponent<Skybox>();
    skybox->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
    skybox->SetMaterial(cache->GetResource<Material>("Materials/Skybox.xml"));



    CreateScenery();



    //SpawnSamplePhysicsSphere(scene_, Vector3(0, 0, 0));
    //SpawnSamplePhysicsSphere(scene_, Vector3(2, 0, 0));

    //SpawnMaterialsTest(Vector3(0,0,30));
    //SpawnBallSocketTest(Vector3(0, 10, 0));

    //CreatePyramids();

    int numVertical = 1;
    //for (int i = 0; i < numVertical; i++)
    //    SpawnCompound(Vector3(-2,1*i,i + 10));
    //for (int i = 0; i < numVertical; i++)
    //    SpawnConvexHull(Vector3(0, 1*i, i + 10));
    //for(int i = 0; i < numVertical; i++)
    //    SpawnDecompCompound(Vector3(2, 1*i, i + 10));

    //SpawnLinearJointedObject(Vector3(10,1,10));
    
    //SpawnNSquaredJointedObject(Vector3(-10, 10, 10));

    //create scale test
    //SpawnSceneCompoundTest(Vector3(-20, 1, 10));



    // Create the camera. Set far clip to match the fog. Note: now we actually create the camera node outside the scene, because
    // we want it to be unaffected by scene load / save
    cameraNode_ = new Node(context_);
    auto* camera = cameraNode_->CreateComponent<Camera>();
    camera->SetFarClip(500.0f);

    // Set an initial position for the camera scene node above the floor
    cameraNode_->SetPosition(Vector3(0.0f, 5.0f, 0.0));
}
void Physics::CreateInstructions()
{
    auto* cache = GetSubsystem<ResourceCache>();
    auto* ui = GetSubsystem<UI>();

    // Construct new Text object, set string to display and font to use
    auto* instructionText = ui->GetRoot()->CreateChild<Text>();
    instructionText->SetText(
        "Use WASD keys and mouse/touch to move\n"
        "LMB to spawn physics objects\n"
        "F5 to save scene, F7 to load\n"
        "Space to toggle physics debug geometry"
    );
    instructionText->SetFont(cache->GetResource<Font>("Fonts/Anonymous Pro.ttf"), 15);
    // The text has multiple rows. Center them in relation to each other
    instructionText->SetTextAlignment(HA_CENTER);

    // Position the text relative to the screen center
    instructionText->SetHorizontalAlignment(HA_CENTER);
    instructionText->SetVerticalAlignment(VA_CENTER);
    instructionText->SetPosition(0, ui->GetRoot()->GetHeight() / 4);
}

void Physics::SetupViewport()
{
    auto* renderer = GetSubsystem<Renderer>();

    // Set up a viewport to the Renderer subsystem so that the 3D scene can be seen
    SharedPtr<Viewport> viewport(new Viewport(context_, scene_, cameraNode_->GetComponent<Camera>()));
    renderer->SetViewport(0, viewport);
}

void Physics::SubscribeToEvents()
{
    // Subscribe HandleUpdate() function for processing update events
    SubscribeToEvent(E_UPDATE, URHO3D_HANDLER(Physics, HandleUpdate));

    // Subscribe HandlePostRenderUpdate() function for processing the post-render update event, during which we request
    // debug geometry
    SubscribeToEvent(E_POSTRENDERUPDATE, URHO3D_HANDLER(Physics, HandlePostRenderUpdate));

    SubscribeToEvent(E_MOUSEBUTTONUP, URHO3D_HANDLER(Physics, HandleMouseButtonUp));

    SubscribeToEvent(E_MOUSEBUTTONDOWN, URHO3D_HANDLER(Physics, HandleMouseButtonDown));
}

void Physics::MoveCamera(float timeStep)
{
    // Do not move if the UI has a focused element (the console)
    if (GetSubsystem<UI>()->GetFocusElement())
        return;

    auto* input = GetSubsystem<Input>();

    // Movement speed as world units per second
    const float MOVE_SPEED = 20.0f;
    // Mouse sensitivity as degrees per pixel
    const float MOUSE_SENSITIVITY = 0.1f;

    // Use this frame's mouse motion to adjust camera node yaw and pitch. Clamp the pitch between -90 and 90 degrees
    IntVector2 mouseMove = input->GetMouseMove();
    yaw_ += MOUSE_SENSITIVITY * mouseMove.x_;
    pitch_ += MOUSE_SENSITIVITY * mouseMove.y_;
    pitch_ = Clamp(pitch_, -90.0f, 90.0f);

    // Construct new orientation for the camera scene node from yaw and pitch. Roll is fixed to zero
    cameraNode_->SetRotation(Quaternion(pitch_, yaw_, 0.0f));

    // Read WASD keys and move the camera scene node to the corresponding direction if they are pressed
    if (input->GetKeyDown(KEY_W))
        cameraNode_->Translate(Vector3::FORWARD * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_S))
        cameraNode_->Translate(Vector3::BACK * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_A))
        cameraNode_->Translate(Vector3::LEFT * MOVE_SPEED * timeStep);
    if (input->GetKeyDown(KEY_D))
        cameraNode_->Translate(Vector3::RIGHT * MOVE_SPEED * timeStep);



    if (input->GetMouseButtonPress(MOUSEB_LEFT))
        CreatePickTargetNodeOnPhysics();




    if (input->GetKeyPress(KEY_R)) {

        int rando = Random(6);
    }

    if (input->GetKeyPress(KEY_TAB))
    {
        input->SetMouseMode(MM_ABSOLUTE);
        GetSubsystem<Input>()->SetMouseVisible(!GetSubsystem<Input>()->IsMouseVisible());
        GetSubsystem<Input>()->SetMouseGrabbed(!GetSubsystem<Input>()->IsMouseGrabbed());
       
    }


    if (input->GetMouseButtonPress(MOUSEB_RIGHT))
        DecomposePhysicsTree();

    if (input->GetMouseButtonPress(MOUSEB_MIDDLE))
    {
        FireSmallBall();
    }

    if (input->GetKeyPress(KEY_T))
        TransportNode();

    if (input->GetKeyPress(KEY_Y))
        RecomposePhysicsTree();

    if (input->GetKeyPress(KEY_L))
    {
        //mark all physics things dirty
        PODVector<Node*> nodes;
        scene_->GetChildrenWithComponent<NewtonRigidBody>(nodes, true);

        for (Node* node : nodes)
        {
            node->GetComponent<NewtonRigidBody>()->MarkDirty();
        }
        nodes.Clear();
        scene_->GetChildrenWithDerivedComponent<NewtonCollisionShape>(nodes, true);

        for (Node* node : nodes)
        {
            node->GetDerivedComponent<NewtonCollisionShape>()->MarkDirty();
        }
    }

    // Check for loading/saving the scene. Save the scene to the file Data/Scenes/Physics.xml relative to the executable
    // directory
    if (input->GetKeyPress(KEY_F5))
    {
        String filePath = GetSubsystem<FileSystem>()->GetProgramDir();
#if _MSC_VER
        filePath += "../";
#endif
        filePath += "Data/Scenes/Physics.xml";
        File saveFile(context_, filePath, FILE_WRITE);
        scene_->SaveXML(saveFile);


        scene_->GetComponent<UrhoNewtonPhysicsWorld>()->SerializeNewtonWorld("newtonWorldFile.ngd");


    }
    if (input->GetKeyPress(KEY_F7))
    {
        String filePath = GetSubsystem<FileSystem>()->GetProgramDir();
#if _MSC_VER
        filePath += "../";
#endif
        filePath += "Data/Scenes/Physics.xml";
        File loadFile(context_, filePath, FILE_READ);
        scene_->LoadXML(loadFile);
    }

    // Toggle physics debug geometry with space
    if (input->GetKeyPress(KEY_SPACE))
        drawDebug_ = !drawDebug_;
}



void Physics::SpawnSceneCompoundTest(const Vector3& worldPos)
{
    Node* root = scene_->CreateChild();
    root->SetPosition(worldPos);
    const int levelCount = 10;
    const int breadth = 2;
    Node* curNode = root;


    for (int i = 0; i < levelCount; i++)
    {

        curNode = curNode->CreateChild();
        curNode->SetName("SpawnSceneCompoundTest:" + String(i));
        curNode->AddTag("scaleTestCube");
        float rotDelta = 0;// Random(-20.0f, 20.0f);
        curNode->Rotate(Quaternion(rotDelta, rotDelta, rotDelta));
        curNode->SetScale(1.0f);
        curNode->Translate(Vector3(0, 1.5f, 0));

        StaticModel* stMdl = curNode->CreateComponent<StaticModel>();
        stMdl->SetModel(GSS<ResourceCache>()->GetResource<Model>("Models/Cone.mdl"));
        stMdl->SetMaterial(GSS<ResourceCache>()->GetResource<Material>("Materials/Stone.xml"));
        stMdl->SetCastShadows(true);
        //if (i == 0) {
            NewtonRigidBody* rigBody = curNode->CreateComponent<NewtonRigidBody>();
            rigBody->SetMassScale(1.0f);
        //}
        NewtonCollisionShape* colShape = curNode->CreateComponent<NewtonCollisionShape_Cone>();
        colShape->SetRotationOffset(Quaternion(0, 0, 90));

    }
}



void Physics::SpawnObject()
{
    auto* cache = GetSubsystem<ResourceCache>();
    Node* firstNode = nullptr;
    Node* prevNode = nullptr;
    bool isFirstNode = true;
    for (int i = 0; i < 2; i++) {


        // Create a smaller box at camera position
        Node* boxNode;
        if (prevNode)
            boxNode = prevNode->CreateChild();
        else {
            boxNode = scene_->CreateChild();
            firstNode = boxNode;
        }
        prevNode = boxNode;
        const float range = 3.0f;


        boxNode->SetWorldPosition(cameraNode_->GetWorldPosition() + Vector3(Random(-1.0f,1.0f) * range,Random(-1.0f, 1.0f)* range, Random(-1.0f, 1.0f)* range));
        boxNode->SetRotation(cameraNode_->GetRotation());
        boxNode->SetScale(1.0f);

        auto* boxObject = boxNode->CreateComponent<StaticModel>();
        boxObject->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
        boxObject->SetMaterial(cache->GetResource<Material>("Materials/StoneEnvMapSmall.xml"));
        boxObject->SetCastShadows(true);


         // Create physics components, use a smaller mass also
         auto* body = boxNode->CreateComponent<NewtonRigidBody>();
         body->SetMassScale(0.1f);
        
        

        isFirstNode = false;
        //body->SetContinuousCollision(true);
        auto* shape = boxNode->CreateComponent<NewtonCollisionShape_Box>();

        const float OBJECT_VELOCITY = 20.0f;

        // Set initial velocity for the NewtonRigidBody based on camera forward vector. Add also a slight up component
        // to overcome gravity better
       // body->SetLinearVelocity(cameraNode_->GetRotation() * Vector3(0.0f, 0.25f, 1.0f) * OBJECT_VELOCITY);

    }

}


void Physics::CreatePyramids()
{
    int size = 16;
    float horizontalSeperation = 2.0f;
    //create pyramids
    const int numIslands = 0;
    for (int x2 = -numIslands; x2 <= numIslands; x2++)
        for (int y2 = -numIslands; y2 <= numIslands; y2++)
        {
            for (int y = 0; y < size; ++y)
            {
                for (int x = -y; x <= y; ++x)
                {
                    Node* node = SpawnSamplePhysicsSphere(scene_, Vector3((float)x*horizontalSeperation, -(float)y + float(size), 0.0f) + Vector3(x2, 0, y2)*50.0f);
                }
            }
        }
}




void Physics::SpawnConvexHull(const Vector3& worldPos)
{
    auto* cache = GetSubsystem<ResourceCache>();

    // Create a smaller box at camera position
    Node* boxNode = scene_->CreateChild();

    boxNode->SetWorldPosition(worldPos);
    boxNode->SetScale(1.0f);

    auto* boxObject = boxNode->CreateComponent<StaticModel>();
    boxObject->SetModel(cache->GetResource<Model>("Models/Mushroom.mdl"));
    boxObject->SetMaterial(cache->GetResource<Material>("Materials/Mushroom.xml"));
    boxObject->SetCastShadows(true);


    // Create physics components, use a smaller mass also
    auto* body = boxNode->CreateComponent<NewtonRigidBody>();
    body->SetMassScale(1.0f);

    auto* shape = boxNode->CreateComponent<NewtonCollisionShape_ConvexHull>();


}



void Physics::SpawnCompound(const Vector3& worldPos)
{
    auto* cache = GetSubsystem<ResourceCache>();

    // Create a smaller box at camera position
    Node* boxNode = scene_->CreateChild();

    boxNode->SetWorldPosition(worldPos);
    boxNode->SetScale(1.0f);

    auto* boxObject = boxNode->CreateComponent<StaticModel>();
    boxObject->SetModel(cache->GetResource<Model>("Models/Mushroom.mdl"));
    boxObject->SetMaterial(cache->GetResource<Material>("Materials/Mushroom.xml"));
    boxObject->SetCastShadows(true);


    // Create physics components, use a smaller mass also
    auto* body = boxNode->CreateComponent<NewtonRigidBody>();
    body->SetMassScale(1.0f);

    auto* shape = boxNode->CreateComponent<NewtonCollisionShape_ConvexHullCompound>();

}




void Physics::SpawnDecompCompound(const Vector3& worldPos)
{
    auto* cache = GetSubsystem<ResourceCache>();

    // Create a smaller box at camera position
    Node* boxNode = scene_->CreateChild();

    boxNode->SetWorldPosition(worldPos);
    boxNode->SetScale(1.0f);

    auto* boxObject = boxNode->CreateComponent<StaticModel>();
    boxObject->SetModel(cache->GetResource<Model>("Models/Mushroom.mdl"));
    boxObject->SetMaterial(cache->GetResource<Material>("Materials/Mushroom.xml"));
    boxObject->SetCastShadows(true);


    // Create physics components, use a smaller mass also
    auto* body = boxNode->CreateComponent<NewtonRigidBody>();
    body->SetMassScale(1.0f);

    auto* shape = boxNode->CreateComponent<NewtonCollisionShape_ConvexDecompositionCompound>();
}

void Physics::SpawnNSquaredJointedObject(Vector3 worldPosition)
{
    //lets joint spheres together with a distance limiting joint.
    const float dist = 5.0f;

    const int numSpheres = 25;

    PODVector<Node*> nodes;
    //make lots of spheres
    for (int i = 0; i < numSpheres; i++)
    {
        nodes += SpawnSamplePhysicsSphere(scene_, worldPosition - Quaternion(Random()*360.0f, Random()*360.0f, Random()*360.0f) * (Vector3::FORWARD*dist));
    }


    //connect them all O(n*n) joints
    for (Node* node : nodes)
    {
        for (Node* node2 : nodes)
        {
            if (node2 == node)
                continue;

            NewtonFixedDistanceConstraint* constraint = node->CreateComponent<NewtonFixedDistanceConstraint>();
            constraint->SetOtherBody(node2->GetComponent<NewtonRigidBody>());
        }
    }
}


void Physics::SpawnLinearJointedObject(Vector3 worldPosition)
{
    //lets joint spheres together with a distance limiting joint.
    const float dist = 0.5f;

    const int numSpheres = 25;

    PODVector<Node*> nodes;
    //make lots of spheres
    for (int i = 0; i < numSpheres; i++)
    {
        nodes += SpawnSamplePhysicsBox(scene_, worldPosition + Vector3(0,i*dist,0), Vector3(dist, dist, dist)*0.5f);

        if (i > 0) {
            NewtonFixedDistanceConstraint* constraint = nodes[i - 1]->CreateComponent<NewtonFixedDistanceConstraint>();
            constraint->SetOtherBody(nodes[i]->GetComponent<NewtonRigidBody>());
        }
    }
}



void Physics::SpawnMaterialsTest(Vector3 worldPosition)
{

    Node* ramp = SpawnSamplePhysicsBox(scene_, worldPosition, Vector3(100, 1, 100));
    ramp->Rotate(Quaternion(-45.0f, 0, 0));
    ramp->Translate(Vector3(0, 50, 0), TS_WORLD);
    ramp->GetComponent<NewtonRigidBody>()->SetMassScale(0);


    for (int i = 0; i < 5; i++)
    {
        Node* box = SpawnSamplePhysicsBox(scene_, ramp->GetWorldPosition() + Vector3(-2.5 + float(i)*1.1f, 2, 0), Vector3::ONE);
        NewtonPhysicsMaterial* mat;
        String physMatName = "PhysicsMaterials/Rock.xml";
        if(i > 2)
            physMatName = "PhysicsMaterials/Ice.xml";

        mat = GSS<ResourceCache>()->GetResource<NewtonPhysicsMaterial>(physMatName);
        NewtonCollisionShape* collisionShape = box->GetDerivedComponent<NewtonCollisionShape>();
        collisionShape->SetPhysicsMaterial(mat);
    }
}



void Physics::SpawnBallSocketTest(Vector3 worldPosition)
{
    //lets joint spheres together with a distance limiting joint.
    const float dist = 5.0f;

    const int numSpheres = 25;


    Node* sphere1 =  SpawnSamplePhysicsSphere(scene_, worldPosition);
    Node* sphere2 = SpawnSamplePhysicsSphere(scene_, worldPosition + Vector3(0,-1.0, 0));
    sphere1->GetComponent<NewtonRigidBody>()->SetMassScale(0);
    NewtonBallAndSocketConstraint* constraint = sphere1->CreateComponent<NewtonBallAndSocketConstraint>();
    constraint->SetOtherBody(sphere2->GetComponent<NewtonRigidBody>());




}

void Physics::FireSmallBall()
{
    Node* sphere = SpawnSamplePhysicsSphere(scene_, cameraNode_->GetWorldPosition());
    //sphere->GetComponent<NewtonRigidBody>()->SetLinearVelocity(cameraNode_->GetWorldDirection() * 10.0f);
    //sphere->GetComponent<NewtonRigidBody>()->SetContinuousCollision(true);
    //sphere->GetComponent<NewtonRigidBody>()->SetLinearDamping(0.1f);
    sphere->GetComponent<NewtonRigidBody>()->SetMassScale(Random(1.0f, 10.0f));


    //if (Random(2) == 0)
    //{
    //    sphere->AddTag("bulletball_linearDamp");
    //    sphere->GetComponent<NewtonRigidBody>()->SetLinearDamping(0.5);
    //}
    //else
    //{
    //    sphere->AddTag("bulletball_customDamp");
    //    sphere->GetComponent<NewtonRigidBody>()->SetLinearDamping(0);

    //}
}

void Physics::HandleUpdate(StringHash eventType, VariantMap& eventData)
{
    using namespace Update;

    // Take the frame time step, which is stored as a float
    float timeStep = eventData[P_TIMESTEP].GetFloat();

    // Move the camera, scale movement with time step
    MoveCamera(timeStep);

    UpdatePickPull();


    //debug bulletball_linearDamp ball
    PODVector<Node*> bulletBalls;
    scene_->GetNodesWithTag(bulletBalls, "bulletball_linearDamp");
    if (bulletBalls.Size()) {
        Node* bulletBall = bulletBalls[0];

        float sampleCountFactor = ((1000.0 / 60.0)/(GSS<Engine>()->GetUpdateTimeGoalMs()));

        if(worldPosHistory_.Size() < 128*sampleCountFactor)
            worldPosHistory_.Push((bulletBall->GetWorldPosition() - cameraNode_->GetWorldPosition()).Length());


        ui::PlotLines("bulletball_linearDamp", &worldPosHistory_[0], worldPosHistory_.Size(), 0, nullptr, FLT_MAX, FLT_MAX, ImVec2(400,200));
    }

    //debug bulletball_linearDamp ball
   // PODVector<Node*> bulletBalls;
    scene_->GetNodesWithTag(bulletBalls, "bulletball_customDamp");
    if (bulletBalls.Size()) {
        Node* bulletBall = bulletBalls[0];

        Vector3 vel = bulletBall->GetComponent<NewtonRigidBody>()->GetVelocity();

            Vector3 dragForce = -vel.Normalized()*(vel.LengthSquared())*0.005f;


            bulletBall->GetComponent<NewtonRigidBody>()->ResetForces();
            bulletBall->GetComponent<NewtonRigidBody>()->AddWorldForce(dragForce);


        float sampleCountFactor = ((1000.0 / 60.0) / (GSS<Engine>()->GetUpdateTimeGoalMs()));

        if (worldPosHistory2_.Size() < 128 * sampleCountFactor)
            worldPosHistory2_.Push((bulletBall->GetWorldPosition() - cameraNode_->GetWorldPosition()).Length());


        ui::PlotLines("bulletball_customDamp", &worldPosHistory2_[0], worldPosHistory2_.Size(), 0, nullptr, FLT_MAX, FLT_MAX, ImVec2(400, 200));
    }


    //debug compound
    //PODVector<Node*> dest;
    //scene_->GetNodesWithTag(dest, "scaleTestCube");

    //for (Node* node : dest)
    //{
    //    URHO3D_LOGINFO(node->GetName());
    //    if (node->HasComponent<NewtonRigidBody>())
    //    {
    //        NewtonRigidBody* rigBody = node->GetComponent<NewtonRigidBody>();
    //        URHO3D_LOGINFO("    Rigid Body: " + String((unsigned)(void*)rigBody->GetNewtonBody()));
    //        if (rigBody->GetNewtonBody())
    //        {
    //            URHO3D_LOGINFO("     Collision: " + String((unsigned)(void*)NewtonBodyGetCollision(rigBody->GetNewtonBody())));
    //        }
    //    }
    //    PODVector<NewtonCollisionShape*> colShapes;
    //    node->GetDerivedComponents(colShapes);

    //    for (NewtonCollisionShape* col : colShapes)
    //    {
    //       // URHO3D_LOGINFO("    Internal: " + String((unsigned)(void*)NewtonBodyGetCollision(rigBody->GetNewtonBody())));

    //    }
    //}

}

void Physics::HandlePostRenderUpdate(StringHash eventType, VariantMap& eventData)
{
    // If draw debug mode is enabled, draw physics debug geometry. Use depth test to make the result easier to interpret
    if (drawDebug_) {
        scene_->GetComponent<UrhoNewtonPhysicsWorld>()->DrawDebugGeometry(scene_->GetComponent<DebugRenderer>(), false);
        GSS<VisualDebugger>()->DrawDebugGeometry(scene_->GetComponent<DebugRenderer>());
    }
}

void Physics::DecomposePhysicsTree()
{
    PODVector<RayQueryResult> res;
    Ray ray(cameraNode_->GetWorldPosition(), cameraNode_->GetWorldDirection());
    RayOctreeQuery querry(res, ray);


    scene_->GetComponent<Octree>()->Raycast(querry);

    if (res.Size() > 1) {

        PODVector<Node*> children;
        res[1].node_->GetChildren(children, true);

        //GSS<VisualDebugger>()->AddOrb(res[1].node_->GetWorldPosition(), 1.0f, Color::RED);

        //for (auto* child : children) {
        //    GSS<VisualDebugger>()->AddOrb(child->GetWorldPosition(), 1.0f, Color(Random(), Random(), Random()));
        //    child->SetParent(scene_);
        //}


        res[1].node_->SetParent(scene_);
    }
}

void Physics::RecomposePhysicsTree()
{

    PODVector<Node*> nodes = scene_->GetChildrenWithTag("scaleTestCube");

    for (int i = 1; i < nodes.Size(); i++) {
        nodes[i]->SetParent(nodes[0]);
    }

}


void Physics::TransportNode()
{
    RayQueryResult res = GetCameraPickNode();

    if (res.node_) {
        PODVector<Node*> children;
        if (res.node_->GetName() == "Floor")
            return;

        res.node_->SetWorldPosition(res.node_->GetWorldPosition() + Vector3(Random(), Random()+1.0f, Random())*1.0f);
        //res[1].node_->SetWorldRotation(Quaternion(Random()*360.0f, Random()*360.0f, Random()*360.0f));
    }
}

void Physics::HandleMouseButtonUp(StringHash eventType, VariantMap& eventData)
{
    ReleasePickTargetOnPhysics();
}

void Physics::HandleMouseButtonDown(StringHash eventType, VariantMap& eventData)
{

}

RayQueryResult Physics::GetCameraPickNode()
{
    PODVector<RayQueryResult> res;
    Ray ray(cameraNode_->GetWorldPosition(), cameraNode_->GetWorldDirection());
    RayOctreeQuery querry(res, ray);
    scene_->GetComponent<Octree>()->Raycast(querry);

    if (res.Size() > 1) {
        return res[1];
    }
    return RayQueryResult();
}



void Physics::CreateScenery()
{
    ResourceCache* cache = GSS<ResourceCache>();
    // Create a floor object, 1000 x 1000 world units. Adjust position so that the ground is at zero Y
    Node* floorNode = scene_->CreateChild("Floor");
    floorNode->SetPosition(Vector3(0.0f, -0.5f, 0.0f));
    //floorNode->SetScale(Vector3(1000.0f, 1.0f, 1000.0f));
    auto* floorObject = floorNode->CreateComponent<StaticModel>();
    floorObject->SetModel(cache->GetResource<Model>("Models/Box.mdl"));
    floorObject->SetMaterial(cache->GetResource<Material>("Materials/StoneTiled.xml"));

    // Make the floor physical by adding NewtonRigidBody and NewtonCollisionShape components. The NewtonRigidBody's default
    // parameters make the object static (zero mass.) Note that a NewtonCollisionShape by itself will not participate
    // in the physics simulation
    //NewtonRigidBody* body = floorNode->CreateComponent<NewtonRigidBody>();
    //body->SetMassScale(0.0f);
    auto* shape = floorNode->CreateComponent<NewtonCollisionShape_Box>();
    //shape = floorNode->CreateComponent<NewtonCollisionShape_Box>();
    //shape->SetPositionOffset(Vector3(1, 0, 0));
    // Set a box shape of size 1 x 1 x 1 for collision. The shape will be scaled with the scene node scale, so the
    // rendering and physics representation sizes should match (the box model is also 1 x 1 x 1.)


}

void Physics::CreatePickTargetNodeOnPhysics()
{
    RayQueryResult res = GetCameraPickNode();
    if (res.node_) {
        if (res.node_->GetName() == "Floor")
            return;
        if (!res.node_->HasComponent<NewtonRigidBody>())
            return;


        Node* pickTarget = cameraNode_->CreateChild("PickTarget");
        pickTarget->SetWorldPosition(res.position_);
        pickPullNode = res.node_;

        if (res.node_->GetChild("PickSource"))
        {
            res.node_->GetChild("PickSource")->SetWorldPosition(res.position_);

        }
        else
        {
            res.node_->CreateChild("PickSource");
            res.node_->GetChild("PickSource")->SetWorldPosition(res.position_);
        }


        //make a kinematics joint
        NewtonKinematicsConstraint* constraint = pickPullNode->CreateComponent<NewtonKinematicsConstraint>();

    }
}

void Physics::ReleasePickTargetOnPhysics()
{
    if (pickPullNode)
    {
        pickPullNode->RemoveChild(pickPullNode->GetChild("PickSource"));
        NewtonRigidBody* rigBody = pickPullNode->GetComponent<NewtonRigidBody>();
        if (rigBody)
        {
            rigBody->ResetForces();
        }
        pickPullNode->RemoveComponent<NewtonKinematicsConstraint>();
        pickPullNode = nullptr;
    }

    cameraNode_->RemoveChild(cameraNode_->GetChild("PickTarget"));
}
void Physics::UpdatePickPull()
{
    Node* pickTarget = cameraNode_->GetChild("PickTarget");
    if (!pickTarget)
        return;
    if (!pickPullNode)
        return;


    Node* pickSource = pickPullNode->GetChild("PickSource");

    if (!pickSource)
        return;

    pickPullNode->GetComponent<NewtonKinematicsConstraint>()->SetTargetPosition(pickTarget->GetWorldPosition());


    //Vector3 delta = (pickTarget->GetWorldPosition() - pickSource->GetWorldPosition());

    //float forceFactor = delta.Length()*100.0f*rigBody->GetEffectiveMass();
    //float cuttoff = 10.0f;



    //Vector3 netForce;
    //netForce = delta * forceFactor;
    //netForce -= scene_->GetComponent<UrhoNewtonPhysicsWorld>()->GetGravity();

    ////URHO3D_LOGINFO("picker" + String((unsigned)(void*)rigBody->GetNewtonBody()));

    //rigBody->ResetForces();
    //rigBody->AddWorldForce(netForce);
    ////rigBody->AddImpulse(Vector3::ZERO, delta * 100000.0f);

}
