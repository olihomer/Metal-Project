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
    auto components = entityManager_.listEntityComponents(handle);
    componentManager_.deleteComponents(components);
    entityManager_.deleteEntity(handle);
}


bool World::hasComponent(ECS::Handle<ECS::Entity> entity, ECS::ComponentType type)
{
    return entityManager_.hasComponent(entity, type);
}
