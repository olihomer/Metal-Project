//
//  World.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef World_hpp
#define World_hpp

#include <vector>
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
    ECS::Handle<ECS::Entity> newEntity(std::string name = "");
    template<typename T>
    ECS::Handle<T> addComponent(ECS::Handle<ECS::Entity> entity);
    template<typename T>
    void deleteComponent(ECS::Handle<ECS::Entity> entity, ECS::Handle<T> handle);

    bool entityHasComponent(ECS::Handle<ECS::Entity> entity, ECS::ComponentType type);
    
    template<typename T>
    bool entityHasComponent(ECS::Handle<ECS::Entity> entity);

    void deleteEntity(ECS::Handle<ECS::Entity> handle);

    void printEntities(){entityManager_.entities_.print();}
    void printComponents(){componentManager_.lights_.print();}

private:
    ECS::EntityManager entityManager_;
    ECS::ComponentManager componentManager_;
};

template<typename T>
ECS::Handle<T> World::addComponent(ECS::Handle<ECS::Entity> entity)
{
    if(entityHasComponent<T>(entity))throw(std::runtime_error("Entity cannot have more than one instance of a given component type"));
    auto component = componentManager_.addComponent<T>(entity);
    entityManager_.registerComponent<T>(entity, component);
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
bool World::entityHasComponent(ECS::Handle<ECS::Entity> entity)
{
    return entityManager_.entityHasComponent<T>(entity);
}


#endif /* World_hpp */
