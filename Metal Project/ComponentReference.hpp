//
//  ComponentReference.hpp
//  Metal Project
//
//  Created by Oliver Homer on 26/10/2026.
//

#ifndef ComponentReference_hpp
#define ComponentReference_hpp

#include "Light.hpp"
#include "Mesh.hpp"
#include "RigidBody.hpp"
#include "Transform.hpp"
#include <cstdint>

namespace ECS {

enum class ComponentType {
    Light,
    Mesh,
    RigidBody,
    Transform
};

struct ComponentReference
{
    ComponentType type;
    std::uint32_t slotIndex;
    std::uint32_t generation;
};

template<typename T>
ComponentReference referenceFromHandle(ECS::Handle<T> handle)
{
    ComponentReference temp;
    temp.generation = handle.generation;
    temp.slotIndex = handle.slotIndex;
    
    if constexpr (std::is_same_v<T, Light>) temp.type = ComponentType::Light;
    if constexpr (std::is_same_v<T, Mesh>) temp.type = ComponentType::Mesh;
    if constexpr (std::is_same_v<T, RigidBody>) temp.type = ComponentType::RigidBody;
    if constexpr (std::is_same_v<T, Transform>) temp.type = ComponentType::Transform;
    
    return temp;
}

}

#endif /* ComponentReference_hpp */
