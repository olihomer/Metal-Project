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
        /*world_.addComponent<RigidBody>(entities[i],getPhysics("cuboid"),1);

        RigidBody* rigidbody = world_.getComponent<RigidBody>(entities[i]);
        if(rigidbody)rigidbody->setCollider(ColliderType::Sphere, 1.0);

        rigidbody->angularMomentum = Vector3D(0,0,0);
        rigidbody->velocity = Vector3D(0,0,0);*/
    }
    world_.addComponent<Transform>(entities[0],Vector3D(0,0,0));
    world_.addComponent<Transform>(entities[1],Vector3D(3.5,2,3));

    
    auto* trans1 = world_.getComponent<Transform>(entities[0]);
    auto* trans2 = world_.getComponent<Transform>(entities[1]);
    
    //trans1->rotation_.setRotationAroundAxis(90 * DEG_RAD, Vector3D(0,1,0.3));

    trans2->rotation_.setRotationAroundAxis(45 * DEG_RAD, Vector3D(1,1,1));
    
    Cuboid cuboid1{trans1->position_,Vector3D(1.0f,2.0f,3.0f),trans1->rotation_};
    Cuboid cuboid2{trans2->position_,Vector3D(1.0f,2.0f,3.0f),trans2->rotation_};
        
    if(!cuboid1.isColliding(cuboid2))
        std::cout << "No collision" << std::endl;
    else
        std::cout << "COLLISION!" << std::endl;
    
    
    
    auto groundPlane = world_.newEntity("Ground");
    world_.addComponent<Transform>(groundPlane, Vector3D(0,0,0));
    world_.addComponent<RigidBody>(groundPlane);
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
