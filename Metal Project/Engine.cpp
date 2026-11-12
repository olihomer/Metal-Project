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
    
    auto myEntity = world_.newEntity("Entity");

    loadMeshAsset("translated_cuboid_2x4x6.obj", "cuboid", 1.0);
    
    world_.addComponent<Transform>(myEntity,Vector3D(0,0,0));
    world_.addComponent<Mesh>(myEntity,getMeshReference("cuboid"));
    world_.addComponent<RigidBody>(myEntity,getPhysics("cuboid"),1);

    
    RigidBody* rigidbody = world_.getComponent<RigidBody>(myEntity);
    if(rigidbody)rigidbody->setCollider(ColliderType::Sphere, 0.1f);
    
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

void Engine::loadMeshAsset(std::string filename, std::string assetName, double scaleFactor)
{
    if(meshLookup_.find(assetName)!=meshLookup_.end())
    {
        throw std::runtime_error("Duplicate asset name");
    }
    MeshAsset temp(filename, assetName, scaleFactor);
    renderSystem_.prepareMeshBuffers(temp);
    meshAssets_.push_back(temp);
    meshLookup_[assetName] = meshAssets_.size() - 1;
}


std::pair<std::size_t,std::size_t> Engine::getMeshReference(std::string assetName)
{
    if(meshLookup_.find(assetName)==meshLookup_.end())
    {
        throw std::runtime_error("Asset name not found");
    }
    return std::pair<std::size_t,std::size_t>(meshLookup_[assetName],meshAssets_[meshLookup_[assetName]].gpuMeshIndex_);
}


MeshAsset::physics Engine::getPhysics(std::string assetName)
{
    if(meshLookup_.find(assetName)==meshLookup_.end())
    {
        throw std::runtime_error("Asset name not found");
    }
    return meshAssets_[meshLookup_[assetName]].physics_;
    
}
