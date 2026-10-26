//
//  ComponentManager.cpp
//  Metal Project
//
//  Created by Oliver Homer on 26/10/2026.
//

#include "ComponentManager.hpp"

namespace ECS
{

void ComponentManager::deleteComponents(const std::vector<ComponentReference>& components)
{
    for(auto component:components)
    {
        switch(component.type)
        {
            case ComponentType::Light:
                lights_.deleteElement(Handle<Light>(component.slotIndex,component.generation));
                break;
            case ComponentType::Mesh:
                meshes_.deleteElement(Handle<Mesh>(component.slotIndex,component.generation));
                break;
            case ComponentType::RigidBody:
                rigidBodies_.deleteElement(Handle<RigidBody>(component.slotIndex,component.generation));
                break;
            case ComponentType::Transform:
                transforms_.deleteElement(Handle<Transform>(component.slotIndex,component.generation));
                break;
        }
    }
}




}
