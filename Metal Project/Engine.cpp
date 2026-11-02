//
//  Engine.cpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#include "Engine.hpp"


void Engine::Update(double dt)
{
    dt = std::clamp(dt, 0.0, 0.25);
    
    accumulator_ += dt;
    
    constexpr double fixedStep = 1.0 / 60.0;
    
    while(accumulator_ >= fixedStep)
    {
        physicsSystem_.updatePhysics(world_, fixedStep);
        accumulator_ -= fixedStep;
    }
}


void Engine::RunApp()
{
    std::cout << "RunApp()" << std::endl;
    
    auto myEntity = world_.newEntity("Dog");

    world_.addComponent<Transform>(myEntity,Vector3D(0,20,3));
    world_.addComponent<RigidBody>(myEntity);
    world_.addComponent<Mesh>(myEntity);
    
    RigidBody* rigidbody = world_.getComponent<RigidBody>(myEntity);
    if(rigidbody)rigidbody->setCollider(ColliderType::Sphere, 1.0f);

    Mesh* mesh = world_.getComponent<Mesh>(myEntity);
    if(mesh)mesh->vertices = {Vector3D(-0.1f,-0.1f,0.0f),Vector3D(0.1f,-0.1f,0.0f),Vector3D(0.0f,0.1f,0.0f)};
    
    
    auto myEntity2 = world_.newEntity("Cat");

    world_.addComponent<Transform>(myEntity2,Vector3D(-5,15,3));
    world_.addComponent<RigidBody>(myEntity2);
    world_.addComponent<Mesh>(myEntity2);
    
    rigidbody = world_.getComponent<RigidBody>(myEntity2);
    if(rigidbody)rigidbody->setCollider(ColliderType::Sphere, 1.0f);

    mesh = world_.getComponent<Mesh>(myEntity2);
    if(mesh)mesh->vertices = {Vector3D(-0.1f,-0.1f,0.0f),Vector3D(0.1f,-0.1f,0.0f),Vector3D(0.0f,0.1f,0.0f)};
}
