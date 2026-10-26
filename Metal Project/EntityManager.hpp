//
//  EntityManager.hpp
//  Metal Project
//
//  Created by Oliver Homer on 24/10/2026.
//

#ifndef EntityManager_hpp
#define EntityManager_hpp

#include "Pool.hpp"
#include "Entity.hpp"

class World;

namespace ECS {

class EntityManager
{
private:
    ECS::Handle<ECS::Entity> newEntity(std::string name);
    void deleteEntity(ECS::Handle<ECS::Entity>);
    
    template<typename T>
    void registerComponent(ECS::Handle<ECS::Entity> entity, ECS::Handle<T> component);
    
    Pool<Entity> entities_;
    friend class ::World;
};


template<typename T>
void EntityManager::registerComponent(ECS::Handle<ECS::Entity> entity, ECS::Handle<T> component)
{
    ComponentReference componentReference = referenceFromHandle(component);
    auto& entityReference = entities_.get(entity);
    entityReference.components_.push_back(componentReference);
}

}
#endif /* EntityManager_hpp */
