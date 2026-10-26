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

class World
{
public:
    World(){};
    ECS::Handle<ECS::Entity> newEntity(std::string name = "");
    template<typename T>
    ECS::Handle<T> addComponent(ECS::Handle<ECS::Entity> entity);

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
    auto component = componentManager_.addComponent<T>(entity);
    entityManager_.registerComponent<T>(entity, component);
    return component;
}

#endif /* World_hpp */
