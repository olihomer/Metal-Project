//
//  Engine.cpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#include "Engine.hpp"


void Engine::Update(int64_t dt)
{
    double dtSeconds = (double)dt/pow(10,18);

    accumulator_ += dtSeconds;
    
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
    auto myEntity2 = world_.newEntity("Cat");
    auto myEntity3 = world_.newEntity("Fish");

    world_.addComponent<Transform>(myEntity,Vector3D(1,2,3));
    world_.addComponent<Mesh>(myEntity);
    world_.addComponent<Light>(myEntity);
    
    std::cout << "hasAllComponents: "<< world_.hasAllComponents<Light,Mesh,Transform,RigidBody>(myEntity) << std::endl;
    
    world_.addComponent<Transform>(myEntity2,Vector3D(11,22,33));
    world_.addComponent<Transform>(myEntity3,Vector3D(111,222,333));

    
    world_.printEntities();
    world_.printComponents<Transform>();

    Transform* transform = world_.getComponent<Transform>(myEntity);
    if(transform)
    {
        std::cout << "Obtained transform: ";
        transform->print();
    }
    else
    {
        std::cout << "No transform found" << std::endl;
    }
    
    
    world_.printComponents<Light>();
    world_.printComponents<Transform>();
    world_.printComponents<RigidBody>();
    world_.printComponents<Mesh>();
    
}
