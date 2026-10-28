//
//  EntityManager.cpp
//  Metal Project
//
//  Created by Oliver Homer on 24/10/2026.
//

#include "EntityManager.hpp"


namespace ECS {

Handle<Entity> EntityManager::newEntity(std::string name)
{
    auto handle = entities_.addElement(name);
    entities_.setOwner(handle, handle);
    return handle;
}

void EntityManager::deleteEntity(Handle<Entity> handle)
{
    entities_.deleteElement(handle);
}

const std::vector<ECS::ComponentReference>& EntityManager::listEntityComponents(ECS::Handle<ECS::Entity> entity)
{
    auto& entityReference = entities_.get(entity);
    return entityReference.components_;
}

bool EntityManager::hasComponent(ECS::Handle<ECS::Entity> entity, ComponentType type)
{
    return entities_.get(entity).hasComponent(type);
}



}
