//
//  PhysicsSystem.cpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#include "PhysicsSystem.hpp"



void PhysicsSystem::updatePhysics(World& world, double dt)
{
    std::cout << "updatePhysics " << dt << std::endl;
    
    world.each<Transform,RigidBody>([&](ECS::Handle<ECS::Entity> entity, Transform& transform, RigidBody& rigidbody)
        {updateEntity(entity,transform,rigidbody,dt);});

}


void PhysicsSystem::updateEntity(ECS::Handle<ECS::Entity> entity, Transform& transform, RigidBody& rigidbody, double dt)
{
    std::cout << "updateEntity" << std::endl;
    Vector3D netForce = accumulateForces(rigidbody);
    Vector3D acceleration = netForce * (1.0f/rigidbody.mass);

    //Symplectic Euler Integration
    
    rigidbody.velocity = rigidbody.velocity + acceleration * dt;
    transform.position_ = transform.position_ + rigidbody.velocity;
    std::cout << transform.position_ << std::endl;
}


Vector3D PhysicsSystem::accumulateForces(RigidBody& rigidbody)
{
    return Vector3D(0,- gravity * rigidbody.mass,0);
}
