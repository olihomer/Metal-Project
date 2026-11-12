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

struct Contact
{
    ECS::Handle<ECS::Entity> entity1;
    ECS::Handle<ECS::Entity> entity2;
    Vector3Dd worldLocation;
};

class PhysicsSystem
{
public:
    void updatePhysics(World& world, double dt);
    
    double timeElapsed{};
    double magT0{};
    double magT1{};
private:
    static constexpr float gravity = 9.81;
    static constexpr float density = 1.225;
    static constexpr float minVelocity = 0.005f;
    static constexpr float stickingThreshold = 0.05f;
    static constexpr float zeroRestitutionThreshold = 0.005f;
        
    void updateEntity(ECS::Handle<ECS::Entity> entity, Transform& transform, RigidBody& rigidbody, double dt);
    void resolveCollisions(Transform& transform, RigidBody& rigidbody) const;
    Vector3D accumulateForces(const Transform& transform, RigidBody& rigidbody, double dt) const;
    void integrateVelocities(World& world, double dt);
    void integratePositions(World& world, double dt);
    void integrateEntityVelocities(Transform& transform, RigidBody& rigidbody, double dt);
    void integrateEntityPositions(Transform& transform, RigidBody& rigidbody, double dt);
    std::vector<Contact> detectCollisions(World& world);
    void solveContacts(World& world, std::vector<Contact> contacts, double dt);
    void correctPenetrations(World& world);
    
    bool touchingGround(const Transform& transform, RigidBody& rigidbody) const;
    void applyDamping(RigidBody& rigidbody) const;
};



#endif /* PhysicsSystem_hpp */
