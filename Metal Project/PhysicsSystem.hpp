//
//  PhysicsSystem.hpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#ifndef PhysicsSystem_hpp
#define PhysicsSystem_hpp

#include "World.hpp"
#include <vector>

constexpr float touchingDistance = 0.0001f;
constexpr float collisionThreshold = 0.01f;
constexpr float bounceThreshold = 0.5f;


struct CollisionCandidate
{
    ECS::Handle<ECS::Entity> entity;
    RigidBody* rigidbody;
    Transform* transform;
};

struct ImpulseAccumulator
{
    double normalLinear;
    Vector3Dd tangentialLinear;
    double spinAngular;
    Vector3Dd rollAngular;
};

struct Contact
{
    CollisionCandidate entity1;
    CollisionCandidate entity2;
    Vector3D worldLocation;
    Vector3D worldNormal;
    ImpulseAccumulator impulseAccumulator;
    double penetrationDepth;
    bool isRolling = false;
    
};

struct ForceAccumulator
{
    Vector3D netForce{};
    Vector3Dd netTorque{};
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
    
    std::vector<Contact> contacts_;
        
    void resolveCollisions(Transform& transform, RigidBody& rigidbody) const;
    ForceAccumulator accumulateForces(const Transform& transform, RigidBody& rigidbody, double dt) const;
    void integrateVelocities(World& world, double dt);
    void integratePositions(World& world, double dt);
    void integrateEntityVelocities(Transform& transform, RigidBody& rigidbody, double dt);
    void integrateEntityPositions(Transform& transform, RigidBody& rigidbody, double dt);
    void detectCollisions(World& world);
    void solveContacts(World& world, double dt);
    void solveContactsIterator(World& world, double dt, int iterations);
    void correctPenetrations(World& world);
    
    void applyDamping(RigidBody& rigidbody) const;

};



#endif /* PhysicsSystem_hpp */
