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
    return entities_.addElement();
}

void EntityManager::deleteEntity(Handle<Entity> handle)
{
    entities_.deleteElement(handle);
}

}
