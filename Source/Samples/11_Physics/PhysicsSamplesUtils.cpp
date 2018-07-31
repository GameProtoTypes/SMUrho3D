#include "PhysicsSamplesUtils.h"


Node* SpawnSamplePhysicsSphere(Scene* scene, const Vector3& worldPosition, float radius)
{
        Node* sphere1 = scene->CreateChild();


        Model* sphereMdl = scene->GSS<ResourceCache>()->GetResource<Model>("Models/Sphere.mdl");
        Material* sphereMat = scene->GSS<ResourceCache>()->GetResource<Material>("Materials/Stone.xml");
        
        StaticModel* sphere1StMdl = sphere1->CreateComponent<StaticModel>();
        sphere1StMdl->SetCastShadows(true);
        sphere1StMdl->SetModel(sphereMdl);
        sphere1StMdl->SetMaterial(sphereMat);


        NewtonRigidBody* s1RigBody = sphere1->CreateComponent<NewtonRigidBody>();

        NewtonCollisionShape* s1ColShape = sphere1->CreateComponent<NewtonCollisionShape>();

        s1ColShape->SetSphere(radius);

        sphere1->SetWorldPosition(worldPosition);

        s1RigBody->SetMassScale(1.0f);


        sphere1->SetScale(radius);

        return sphere1;
}

Node* SpawnSamplePhysicsBox(Scene* scene, const Vector3& worldPosition)
{

    Node* box = scene->CreateChild();


    Model* sphereMdl = scene->GSS<ResourceCache>()->GetResource<Model>("Models/Box.mdl");
    Material* sphereMat = scene->GSS<ResourceCache>()->GetResource<Material>("Materials/Stone.xml");

    StaticModel* sphere1StMdl = box->CreateComponent<StaticModel>();
    sphere1StMdl->SetCastShadows(true);
    sphere1StMdl->SetModel(sphereMdl);
    sphere1StMdl->SetMaterial(sphereMat);


    NewtonRigidBody* s1RigBody = box->CreateComponent<NewtonRigidBody>();

    NewtonCollisionShape* s1ColShape = box->CreateComponent<NewtonCollisionShape>();

    s1ColShape->SetBox(Vector3::ONE);

    box->SetWorldPosition(worldPosition);

    s1RigBody->SetMassScale(1.0f);


    return box;
}

