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
    template<typename T>
    void deRegisterComponent(ECS::Handle<ECS::Entity> entity, ECS::Handle<T> component);

    bool entityHasComponent(ECS::Handle<ECS::Entity> entity, ComponentType type);

    template<typename T>
    bool entityHasComponent(ECS::Handle<ECS::Entity> entity);

    const std::vector<ECS::ComponentReference>& listEntityComponents(ECS::Handle<ECS::Entity>);
    
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

template<typename T>
void EntityManager::deRegisterComponent(ECS::Handle<ECS::Entity> entity, ECS::Handle<T> component)
{
    ComponentReference componentReference = referenceFromHandle(component);
    auto& entityComponentsList = entities_.get(entity).components_;
    
    std::size_t index = 0;
    for(ComponentReference component: entityComponentsList)
    {
        if(component == componentReference)
        {
            entityComponentsList.erase(entityComponentsList.begin()+index);
            break;
        }
        index++;
    }
}

template<typename T>
bool EntityManager::entityHasComponent(ECS::Handle<ECS::Entity> entity)
{
    return entities_.get(entity).hasComponent<T>();
}


}
#endif /* EntityManager_hpp */
