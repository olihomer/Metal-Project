//
//  World.cpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#include "World.hpp"

ECS::Handle<ECS::Entity> World::newEntity(std::string name)
{
    return entityManager_.newEntity(name);
}


void World::deleteEntity(ECS::Handle<ECS::Entity> handle)
{
    entityManager_.deleteEntity(handle);
}
