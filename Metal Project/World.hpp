//
//  World.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef World_hpp
#define World_hpp

#include <vector>
#include <functional>
#include "Pool.hpp"
#include "RigidBody.hpp"
#include "Mesh.hpp"
#include "Light.hpp"
#include "Entity.hpp"
#include "EntityManager.hpp"
#include "ComponentManager.hpp"
#include "ComponentReference.hpp"

class World
{
public:
    World(){};
    ECS::Handle<ECS::Entity> newEntity(std::string name = "Unnamed");

    template<typename T>
    void printComponents(){componentManager_.printComponents<T>();}
    template<typename T, typename... Args>
    ECS::Handle<T> addComponent(ECS::Handle<ECS::Entity> entity, Args&&... args);
    template<typename T>
    void deleteComponent(ECS::Handle<ECS::Entity> entity, ECS::Handle<T> handle);
    template<typename T>
    bool hasComponent(ECS::Handle<ECS::Entity> entity);
    template<typename... Ts>
    bool hasAllComponents(ECS::Handle<ECS::Entity> entity);
    template<typename T>
    T* getComponent(ECS::Handle<ECS::Entity> entity);
    template<typename T>
    void each(std::function<void(ECS::Handle<ECS::Entity>,T&)> callback);
    template<typename T1, typename... Ts, typename F>
    void each(F callback);

    bool hasComponent(ECS::Handle<ECS::Entity> entity, ECS::ComponentType type);
    void deleteEntity(ECS::Handle<ECS::Entity> handle);
    void printEntities(){entityManager_.entities_.print();}

private:
    ECS::EntityManager entityManager_;
    ECS::ComponentManager componentManager_;
};

template<typename T, typename... Args>
ECS::Handle<T> World::addComponent(ECS::Handle<ECS::Entity> entity, Args&&... args)
{
    if(hasComponent<T>(entity))throw(std::runtime_error("Entity cannot have more than one instance of a given component type"));
    auto component = componentManager_.addComponent<T>(entity,std::forward<Args>(args)...);
    try{
        entityManager_.registerComponent<T>(entity, component);
    }
    catch(...){
        componentManager_.deleteComponent(component);
        throw;
    }
    return component;
}

template<typename T>
void World::deleteComponent(ECS::Handle<ECS::Entity> entity, ECS::Handle<T> handle)
{
    auto components = entityManager_.listEntityComponents(entity);
    for(auto component:components)
    {
        if(ECS::referenceFromHandle(handle)==component)
        {
            componentManager_.deleteComponent<T>(handle);
            entityManager_.deRegisterComponent<T>(entity, handle);
            return;
        }
    }
    throw(std::runtime_error("Component not found in deleteComponent"));
}

template<typename T>
bool World::hasComponent(ECS::Handle<ECS::Entity> entity)
{
    return entityManager_.hasComponent<T>(entity);
}


template<typename T>
T* World::getComponent(ECS::Handle<ECS::Entity> entity)
{
    if(!entityManager_.hasComponent<T>(entity))return nullptr;
    ECS::ComponentReference component = entityManager_.getComponent<T>(entity);
    ECS::Handle<T> handle = {component.slotIndex,component.generation};
    return componentManager_.getComponent(handle);
}


template<typename T>
void World::each(std::function<void(ECS::Handle<ECS::Entity>,T&)> callback)
{
    componentManager_.each(callback);
}

template<typename... Ts>
bool World::hasAllComponents(ECS::Handle<ECS::Entity> entity)
{
    return entityManager_.hasAllComponents<Ts...>(entity);
}

template<typename T1, typename... Ts, typename F>
void World::each(F callback)
{
    ECS::Pool<T1>& pool = componentManager_.getPool<T1>();
    for(uint32_t i = 0; i < pool.poolSize(); i++)
    {
        auto handle = pool.getHandle(i);
        ECS::Handle<ECS::Entity> owner = pool.getOwner(handle);
        if(hasAllComponents<Ts...>(owner))
            callback(pool.getOwner(handle),pool.get(handle),*getComponent<Ts>(owner)...);
    }
}

#endif /* World_hpp */
