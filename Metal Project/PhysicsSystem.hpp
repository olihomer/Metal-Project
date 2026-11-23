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
constexpr std::string_view faceNames[6] = {"Left","Right","Bottom","Top","Backwards","Forwards"};

struct CollisionCandidate
{
    ECS::Handle<ECS::Entity> entity;
    RigidBody* rigidbody;
    Transform* transform;
};

struct ImpulseAccumulator
{
    double normalLinear{};
    Vector3Dd tangentialLinear{};
    double spinAngular{};
    Vector3Dd rollAngular{};
};

struct Contact
{
    CollisionCandidate entity1{};
    CollisionCandidate entity2{};
    Vector3D worldLocation{};
    Vector3D worldNormal{};
    ImpulseAccumulator impulseAccumulator{};
    double penetrationDepth{};
    bool isRolling = false;
    double targetNormalVelocity{};
    
};

struct ForceAccumulator
{
    Vector3D netForce{};
    Vector3Dd netTorque{};
};

struct Plane
{
    Plane(Vector3D p, Vector3D n){point = p; normal = n.getNormalized();}
    Vector3D point{};
    Vector3D normal{};
    bool pointIsAbove(Vector3D x){return(Vector3D::dot(x-point,normal)>0);}
    bool pointIsBelow(Vector3D x){return(Vector3D::dot(x-point,normal)<0);}
    bool pointOnPlane(Vector3D x){return(Vector3D::dot(x-point,normal)==0);}
};

struct Cuboid
{
    Cuboid(Vector3D point_, Vector3D halfEdge_, Quaternion rotation_):p(point_),halfEdge(halfEdge_),rotation(rotation_){}
    Vector3D p{};
    Vector3D halfEdge{};
    Quaternion rotation{};
    bool isColliding(Cuboid& cuboid2);

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
    std::vector<Contact> detectCollisions(World& world);
    void solveContactsIterator(World& world, std::vector<Contact>& contacts, double dt, int iterations);
    void correctPenetrations(World& world, std::vector<Contact>& contacts) const;
    
    void applyDamping(RigidBody& rigidbody) const;

};



#endif /* PhysicsSystem_hpp */
