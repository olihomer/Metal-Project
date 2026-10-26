//
//  ComponentManager.hpp
//  Metal Project
//
//  Created by Oliver Homer on 26/10/2026.
//

#ifndef ComponentManager_hpp
#define ComponentManager_hpp


#include "Pool.hpp"
#include "Light.hpp"
#include "Mesh.hpp"
#include "RigidBody.hpp"
#include "Transform.hpp"
#include "ComponentReference.hpp"

class World;

namespace ECS {

class Entity;


class ComponentManager
{
private:
    template<typename T>
    ECS::Handle<T> addComponent(ECS::Handle<Entity> entity);
    template<typename T>
    void deleteComponent(ECS::Handle<T> handle);

    void deleteComponents(const std::vector<ComponentReference>& components);
    
    template<typename T>
    Pool<T>& GetPool();
    
    Pool<Light> lights_;
    Pool<Mesh> meshes_;
    Pool<RigidBody> rigidBodies_;
    Pool<Transform> transforms_;
    
    friend class ::World;
};

template<typename T>
ECS::Handle<T> ComponentManager::addComponent(ECS::Handle<Entity> entity)
{
    return GetPool<T>().addElement();
}

template<typename T>
void ComponentManager::deleteComponent(ECS::Handle<T> handle)
{
    return GetPool<T>().deleteElement(handle);
}

template<typename T>
Pool<T>& ComponentManager::GetPool()
{
    if constexpr (std::is_same_v<T, Light>) return lights_;
    else
        if constexpr (std::is_same_v<T, Mesh>) return meshes_;
    else
        if constexpr (std::is_same_v<T, RigidBody>) return rigidBodies_;
    else
        if constexpr (std::is_same_v<T, Transform>) return transforms_;
    else
        static_assert(false, "Unsupported pool type");
}


}

#endif /* ComponentManager_hpp */
