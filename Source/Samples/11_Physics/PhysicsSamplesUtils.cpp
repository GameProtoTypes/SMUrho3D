#include "PhysicsSamplesUtils.h"
#include "Urho3D/Physics/NewtonCollisionShapesDerived.h"

Node* SpawnSamplePhysicsSphere(Node* parentNode, const Vector3& worldPosition, float radius)
{
        Node* sphere1 = parentNode->CreateChild("SamplePhysicsSphere");


        Model* sphereMdl = parentNode->GSS<ResourceCache>()->GetResource<Model>("Models/Sphere.mdl");
        Material* sphereMat = parentNode->GSS<ResourceCache>()->GetResource<Material>("Materials/Stone.xml");
        
        StaticModel* sphere1StMdl = sphere1->CreateComponent<StaticModel>();
        sphere1StMdl->SetCastShadows(true);
        sphere1StMdl->SetModel(sphereMdl);
        sphere1StMdl->SetMaterial(sphereMat);


        NewtonRigidBody* s1RigBody = sphere1->CreateComponent<NewtonRigidBody>();

        NewtonCollisionShape_Sphere* s1ColShape = sphere1->CreateComponent<NewtonCollisionShape_Sphere>();


        sphere1->SetWorldPosition(worldPosition);

        s1RigBody->SetMassScale(1.0f);

        sphere1->SetScale(radius);

        return sphere1;
}

Node* SpawnSamplePhysicsBox(Node* parentNode, const Vector3& worldPosition, const Vector3& size)
{

    Node* box = parentNode->CreateChild();
    box->SetScale(size);

    Model* sphereMdl = parentNode->GSS<ResourceCache>()->GetResource<Model>("Models/Box.mdl");
    Material* sphereMat = parentNode->GSS<ResourceCache>()->GetResource<Material>("Materials/Stone.xml");

    StaticModel* sphere1StMdl = box->CreateComponent<StaticModel>();
    sphere1StMdl->SetCastShadows(true);
    sphere1StMdl->SetModel(sphereMdl);
    sphere1StMdl->SetMaterial(sphereMat);


    NewtonRigidBody* s1RigBody = box->CreateComponent<NewtonRigidBody>();

    NewtonCollisionShape_Box* s1ColShape = box->CreateComponent<NewtonCollisionShape_Box>();

    box->SetWorldPosition(worldPosition);

    s1RigBody->SetMassScale(1.0f);


    return box;
}

