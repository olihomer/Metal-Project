//
//  PhysicsSystem.cpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#include "PhysicsSystem.hpp"



void PhysicsSystem::updatePhysics(World& world, double dt)
{
    std::cout << "updatePhysics " << dt << std::endl;
    
    
    world.each<Transform>([&](ECS::Handle<ECS::Entity> entity, Transform& transform){std::cout<<"Found entity with transform: "<< transform.position_ << std::endl;});

    
}
