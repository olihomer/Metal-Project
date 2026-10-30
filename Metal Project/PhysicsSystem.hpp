//
//  PhysicsSystem.hpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#ifndef PhysicsSystem_hpp
#define PhysicsSystem_hpp

#include "World.hpp"

class PhysicsSystem
{
public:
    void updatePhysics(World& world, double dt);
private:
    static constexpr float gravity = 9.81;
    static constexpr float density = 1.225;
    
    void updateEntity(ECS::Handle<ECS::Entity> entity, Transform& transform, RigidBody& rigidbody, double dt);
    Vector3D accumulateForces(RigidBody& rigidbody);
};



#endif /* PhysicsSystem_hpp */
