//
//  Engine.cpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#include "Engine.hpp"
#include "Transform.hpp"
#include "Matrix4x4.h"
#include "Matrix3x3.hpp"



void Engine::RunApp()
{
    std::cout << "RunApp()" << std::endl;
    
    loadMeshAsset("cuboid_2x4x6.obj", "cuboid", 1.0);

    std::vector<ECS::Handle<ECS::Entity>> entities;
    entities.resize(50);
    
    for(int i = 0; i < 2; i++)
    {
        entities[i] = world_.newEntity("Entity");
        world_.addComponent<Mesh>(entities[i],getMeshReference("cuboid"));
        world_.addComponent<RigidBody>(entities[i],getPhysics("cuboid"),MotionType::Dynamic,1);
        
    }
    
    world_.addComponent<Transform>(entities[0],Vector3D(0,6.0f,0));
    world_.addComponent<Transform>(entities[1],Vector3D(0,12.0f,0));

    auto* trans1 = world_.getComponent<Transform>(entities[0]);
    auto* trans2 = world_.getComponent<Transform>(entities[1]);
    
    trans1->rotation_.setRotationAroundAxis(90 * DEG_RAD, Vector3D(0,1,0));
    trans2->rotation_.setRotationAroundAxis(0 * DEG_RAD, Vector3D(0,0,0));
    
    auto* rb1 = world_.getComponent<RigidBody>(entities[0]);
    auto* rb2 = world_.getComponent<RigidBody>(entities[1]);
    
    rb1->setCollider(ColliderType::Cuboid,0,1.0f,2.0f,3.0f);
    rb2->setCollider(ColliderType::Cuboid,0,1.0f,2.0f,3.0f);
    
    rb1->velocity = Vector3D(0,0.0f,0);
    rb2->velocity = Vector3D(0,0,0);

    
    auto groundPlane = world_.newEntity("Ground");
    world_.addComponent<Transform>(groundPlane, Vector3D(0,0,0));
    world_.addComponent<RigidBody>(groundPlane,MotionType::Static);
    auto* groundRB = world_.getComponent<RigidBody>(groundPlane);

    groundRB->setCollider(ColliderType::GroundPlane);
    

}


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
