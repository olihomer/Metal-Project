//
//  PhysicsSystem.hpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#ifndef PhysicsSystem_hpp
#define PhysicsSystem_hpp

#include "World.hpp"

constexpr float touchingDistance = 0.01f;

class PhysicsSystem
{
public:
    void updatePhysics(World& world, double dt) const;
private:
    static constexpr float gravity = 9.81;
    static constexpr float density = 1.225;
    static constexpr float minVelocity = 0.005f;
    static constexpr float stickingThreshold = 0.05f;
    static constexpr float zeroRestitutionThreshold = 0.005f;
        
    void updateEntity(ECS::Handle<ECS::Entity> entity, Transform& transform, RigidBody& rigidbody, double dt) const;
    void resolveCollisions(Transform& transform, RigidBody& rigidbody) const;
    Vector3D accumulateForces(const Transform& transform, RigidBody& rigidbody, double dt) const;
    bool touchingGround(const Transform& transform, RigidBody& rigidbody) const;
    void applyDamping(RigidBody& rigidbody) const;
};



#endif /* PhysicsSystem_hpp */
