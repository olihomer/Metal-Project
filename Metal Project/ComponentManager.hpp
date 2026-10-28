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
#include <cxxabi.h>

class World;

namespace ECS {

class Entity;


class ComponentManager
{
private:
    template<typename T, typename... Args>
    ECS::Handle<T> addComponent(ECS::Handle<Entity> entity, Args&&... args);
    template<typename T>
    void deleteComponent(ECS::Handle<T> handle);

    void deleteComponents(const std::vector<ComponentReference>& components);
    
    template<typename T>
    Pool<T>& getPool();
    
    template<typename T>
    void printComponents();
    
    Pool<Light> lights_;
    Pool<Mesh> meshes_;
    Pool<RigidBody> rigidBodies_;
    Pool<Transform> transforms_;
    
    friend class ::World;
};

template<typename T>
void ComponentManager::printComponents()
{
    if(getPool<T>().elements_.size() == 0)
    {
        std::cout << "No components of type " << typeid(T).name() << std::endl;
    }
    for(auto& element: getPool<T>().elements_)
        element.print();
}


template<typename T, typename... Args>
ECS::Handle<T> ComponentManager::addComponent(ECS::Handle<Entity> entity, Args&&... args)
{
    auto handle = getPool<T>().addElement(std::forward<Args>(args)...);
    getPool<T>().setOwner(handle,entity);
    return handle;
}

template<typename T>
void ComponentManager::deleteComponent(ECS::Handle<T> handle)
{
    return getPool<T>().deleteElement(handle);
}

template<typename T>
Pool<T>& ComponentManager::getPool()
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
