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

template<typename T>
inline constexpr ComponentType componentType = []
{
    static_assert(false, "Unsupported component");
    return ComponentType{};
}();

template<>
inline constexpr ComponentType componentType<Light> = ComponentType::Light;

template<>
inline constexpr ComponentType componentType<Mesh> = ComponentType::Mesh;

template<>
inline constexpr ComponentType componentType<RigidBody> = ComponentType::RigidBody;

template<>
inline constexpr ComponentType componentType<Transform> = ComponentType::Transform;

struct ComponentReference
{
    ComponentType type;
    std::uint32_t slotIndex;
    std::uint32_t generation;
    bool operator==(ComponentReference& rhs){return(slotIndex==rhs.slotIndex&&generation==rhs.generation&&type==rhs.type);}
};

template<typename T>
ComponentReference referenceFromHandle(ECS::Handle<T> handle)
{
    return {
        componentType<T>,
        handle.slotIndex,
        handle.generation
    };
}

}

#endif /* ComponentReference_hpp */
