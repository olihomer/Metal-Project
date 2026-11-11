//
//  PhysicsSystem.cpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#include "PhysicsSystem.hpp"

constexpr bool showDebug = false;


void PhysicsSystem::updatePhysics(World& world, double dt) const
{
    if(showDebug)std::cout << "updatePhysics " << dt << std::endl;
    
    world.each<Transform,RigidBody>([&](ECS::Handle<ECS::Entity> entity, Transform& transform, RigidBody& rigidbody)
        {updateEntity(entity,transform,rigidbody,dt);});
}


void PhysicsSystem::updateEntity(ECS::Handle<ECS::Entity> entity, Transform& transform, RigidBody& rigidbody, double dt) const
{
    //accumulate and apply forces
    Vector3D netForce = accumulateForces(transform, rigidbody, dt);
    Vector3D acceleration = netForce * (1.0f/rigidbody.mass);
    
    //Symplectic Euler Integration
    
    //linear position
    rigidbody.velocity = rigidbody.velocity + acceleration * dt;
    transform.position_ = transform.position_ + rigidbody.velocity * dt;
    
    //angular position
    Vector3Dd torque(0,0,0);
    Vector3Dd angularAcceleration = rigidbody.inverseBodyInertia * torque;
    angularAcceleration.print("Angular Acc");
    rigidbody.angularVelocity = rigidbody.angularVelocity + angularAcceleration * dt;
    rigidbody.angularVelocity.print("Angular V");
    
    Vector3Dd omega = rigidbody.angularVelocity;
    float speed = omega.magnitude();
    
    if(speed > 1e-6)
    {
        Vector3Dd axis = omega * (1.0f / speed);
        
        Quaternion deltaRotation;
        deltaRotation.setRotationAroundAxis(speed * dt, axis);
        
        transform.rotation_= deltaRotation * transform.rotation_;
        transform.rotation_.normalise();
    }
    
    //resolve collisions
    resolveCollisions(transform, rigidbody);
    
    //apply damping to low velocities
    applyDamping(rigidbody);
    
    if(showDebug)
    {
        std::cout << "Position: " << transform.position_ << std::endl;
        std::cout << "Velocity:" << rigidbody.velocity << std::endl;
    }
}


Vector3D PhysicsSystem::accumulateForces(const Transform& transform, RigidBody& rigidbody, double dt) const
{
    //gravity
    Vector3D gravityForce{};
    if(!touchingGround(transform, rigidbody)) gravityForce = Vector3D(0,- gravity * rigidbody.mass,0);
    
    //air resistance
    float dragMagnitude = 0.5f * density * pow(rigidbody.velocity.magnitude(),2) * rigidbody.dragCoefficient * rigidbody.crossSectionArea;
    Vector3D dragDirection = rigidbody.velocity * -1.0f;
    dragDirection.normalize();

    Vector3D dragForce =  dragDirection * dragMagnitude;

    //friction
    
    Vector3D frictionForce;
    if(touchingGround(transform, rigidbody))
    {
        Vector3D frictionDirection = Vector3D(rigidbody.velocity.x_, 0.0f, rigidbody.velocity.z_) * -1.0f;
        if(frictionDirection.magnitude()<stickingThreshold)
        {
            rigidbody.velocity.x_ = 0;
            rigidbody.velocity.z_ = 0;
        }
        else
        {
            float frictionCap = frictionDirection.magnitude() * rigidbody.mass * (1/dt);
            frictionDirection.normalize();
            float frictionMagnitude = std::min(rigidbody.mu * (float)rigidbody.mass * gravity, frictionCap);
            frictionForce = frictionDirection * frictionMagnitude;
        }
    }
    
    if(showDebug)
    {
        std::cout << "Gravity: " << gravityForce << std::endl;
        std::cout << "Drag: " << dragForce << std::endl;
        std::cout << "Friction: " << frictionForce << std::endl;
    }
    
    return (gravityForce + dragForce + frictionForce);
}


bool PhysicsSystem::touchingGround(const Transform& transform, RigidBody& rigidbody) const
{
    if(rigidbody.collider_.type_ == ColliderType::Sphere)
        return (transform.position_.y_ < (rigidbody.collider_.r_ + touchingDistance));
    
    return false;
}


void PhysicsSystem::resolveCollisions(Transform &transform, RigidBody &rigidbody) const
{
    //Collision with ground
    
    if(touchingGround(transform, rigidbody))
    {
        transform.position_.y_ = rigidbody.collider_.r_; //resolve penetration
        if(rigidbody.velocity.y_ < 0.0f)
        {
            rigidbody.velocity.y_ *= -1.0f * rigidbody.elasticity;
            if(rigidbody.velocity.y_ < zeroRestitutionThreshold)rigidbody.velocity.y_ = 0;
        }
    }
}


void PhysicsSystem::applyDamping(RigidBody &rigidbody) const
{
    if(std::abs(rigidbody.velocity.x_) < minVelocity)rigidbody.velocity.x_ = 0.0f;
    if(std::abs(rigidbody.velocity.y_) < minVelocity)rigidbody.velocity.y_ = 0.0f;
    if(std::abs(rigidbody.velocity.z_) < minVelocity)rigidbody.velocity.z_ = 0.0f;
    
}
