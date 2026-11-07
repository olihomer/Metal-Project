//
//  Engine.cpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#include "Engine.hpp"


void Engine::Update(double dt)
{
    //Physics

    dt = std::clamp(dt, 0.0, 0.25);
    
    accumulator_ += dt;
    
    constexpr double fixedStep = 1.0 / 60.0;

    while(accumulator_ >= fixedStep)
    {
        physicsSystem_.updatePhysics(world_, fixedStep);
        accumulator_ -= fixedStep;
    }

    //Input
    
    Camera* camera = world_.getCamera();
    
    if(inputSystem_.isKeyDown(13))
        camera->Translate(MOVEMENT_SPEED, camera->getForward());
    
    if(inputSystem_.isKeyDown(1))
        camera->Translate(-MOVEMENT_SPEED, camera->getForward());
    
    if(inputSystem_.isKeyDown(0))
        camera->Translate(-MOVEMENT_SPEED, camera->getRight());
    
    if(inputSystem_.isKeyDown(2))
        camera->Translate(MOVEMENT_SPEED, camera->getRight());
    
    if(inputSystem_.isKeyDown(53))
        requestQuit();
}


void Engine::RunApp()
{
    std::cout << "RunApp()" << std::endl;
    
    auto myEntity = world_.newEntity("Dog");

    loadMeshAsset("sphere.obj", "sphere");
    
    world_.addComponent<Transform>(myEntity,Vector3D(-5,5,8));
    world_.addComponent<RigidBody>(myEntity);
    world_.addComponent<Mesh>(myEntity,getMeshReference("sphere"));
    
    RigidBody* rigidbody = world_.getComponent<RigidBody>(myEntity);
    if(rigidbody)rigidbody->setCollider(ColliderType::Sphere, 1.0f);
    
}

void Engine::KeyUp(int keyCode)
{
    inputSystem_.keyUp(keyCode);
}

void Engine::KeyDown(int keyCode)
{
    inputSystem_.keyDown(keyCode);
}

void Engine::MouseMoved(float dx, float dy)
{
    Camera* camera = world_.getCamera();
    camera->Yaw(MOUSE_SENSITIVIY * dx);
    camera->Pitch(MOUSE_SENSITIVIY * dy);
}

void Engine::loadMeshAsset(std::string filename, std::string assetName)
{
    MeshAsset temp(filename, assetName);
    renderSystem_.prepareMeshBuffers(temp);
    meshAssets_.push_back(temp);
    meshLookup_[assetName] = meshAssets_.size() - 1;
}

