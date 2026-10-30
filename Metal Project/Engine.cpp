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

    world_.addComponent<Transform>(myEntity,Vector3D(1,2,3));
    world_.addComponent<RigidBody>(myEntity);
    
}
