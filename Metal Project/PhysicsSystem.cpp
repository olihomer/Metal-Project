//
//  PhysicsSystem.cpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#include "PhysicsSystem.hpp"

constexpr bool showDebug = false;


void PhysicsSystem::updatePhysics(World& world, double dt)
{
    if(showDebug)std::cout << "updatePhysics " << dt << std::endl;
    
    integrateVelocities(world, dt);
    
    auto contacts = detectCollisions(world);
    
    solveContacts(world, contacts, dt);
    
    integratePositions(world, dt);
    
    correctPenetrations(world);
}


void PhysicsSystem::integrateVelocities(World& world, double dt)
{
    world.each<Transform,RigidBody>([&](ECS::Handle<ECS::Entity> entity, Transform& transform, RigidBody& rigidbody)
        {integrateEntityVelocities(transform,rigidbody,dt);});
}

void PhysicsSystem::integratePositions(World& world, double dt)
{
    world.each<Transform,RigidBody>([&](ECS::Handle<ECS::Entity> entity, Transform& transform, RigidBody& rigidbody)
        {integrateEntityPositions(transform,rigidbody,dt);});
}



void PhysicsSystem::integrateEntityPositions(Transform& transform, RigidBody& rigidbody, double dt)
{

    transform.position_ = transform.position_ + rigidbody.velocity * dt;

    Vector3Dd omega = rigidbody.angularVelocity;
    double speed = omega.magnitude();
    
    //update rotation position
    if(speed > 1e-6)
    {
        Vector3Dd axis = omega * (1.0 / speed);
        
        Quaternion deltaRotation;
        deltaRotation.setRotationAroundAxis(speed * dt, axis);
        
        transform.rotation_= deltaRotation * transform.rotation_;
        transform.rotation_.normalise();
    }
    
    //recompute inertia
    rigidbody.updateWorldInertia(transform.rotation_);
    //recompute angular velocity
    rigidbody.angularVelocity = rigidbody.angularMomentum * rigidbody.inverseWorldInertia;

    //track KE
    double transKE = 0.5 * rigidbody.mass * pow(rigidbody.velocity.magnitude(),2);
    double rotKE = 0.5 * Vector3Dd::dot(rigidbody.angularVelocity,rigidbody.angularMomentum);
    double totalKE = transKE + rotKE;
    
    if(showDebug)std::cout << "TransKE: " << transKE << " RotKE: " << rotKE << " TotalKE: " << totalKE << std::endl;
}

void PhysicsSystem::integrateEntityVelocities(Transform& transform, RigidBody& rigidbody, double dt)
{
    //accumulate and apply forces
    ForceAccumulator forces = accumulateForces(transform, rigidbody, dt);
    forces.netForce=Vector3D();
    forces.netTorque=Vector3D();
    Vector3D acceleration = forces.netForce * (1.0f/rigidbody.mass);
        
    //linear velocity
    rigidbody.velocity = rigidbody.velocity + acceleration * dt;
    
    //angular velocity
    rigidbody.updateWorldInertia(transform.rotation_);

    rigidbody.angularMomentum += forces.netTorque * dt;
    rigidbody.angularVelocity = rigidbody.angularMomentum * rigidbody.inverseWorldInertia;
    
}


ForceAccumulator PhysicsSystem::accumulateForces(const Transform& transform, RigidBody& rigidbody, double dt) const
{
    ForceAccumulator forces;
    //gravity
    Vector3D gravityForce{};
    gravityForce = Vector3D(0,- gravity * rigidbody.mass,0);
    
    //air resistance
    float dragMagnitude = 0.5f * density * pow(rigidbody.velocity.magnitude(),2) * rigidbody.dragCoefficient * rigidbody.crossSectionArea;
    Vector3D dragDirection = rigidbody.velocity * -1.0f;
    dragDirection.normalize();

    Vector3D dragForce =  dragDirection * dragMagnitude;
    
    if(showDebug)
    {
        std::cout << "Gravity: " << gravityForce << std::endl;
        std::cout << "Drag: " << dragForce << std::endl;
    }
    
    forces.netForce = gravityForce + dragForce;
    forces.netTorque = Vector3Dd(0,0,0);

    return forces;
}


void PhysicsSystem::applyDamping(RigidBody &rigidbody) const
{
    if(std::abs(rigidbody.velocity.x_) < minVelocity)rigidbody.velocity.x_ = 0.0f;
    if(std::abs(rigidbody.velocity.y_) < minVelocity)rigidbody.velocity.y_ = 0.0f;
    if(std::abs(rigidbody.velocity.z_) < minVelocity)rigidbody.velocity.z_ = 0.0f;
    
}


void PhysicsSystem::solveContacts(World &world, const std::vector<Contact>& contacts, double dt)
{
    //iterate through contacts
    for(auto contact: contacts)
    {
        auto rb1=contact.entity1.rigidbody;
        auto rb2=contact.entity2.rigidbody;
        
        //velocity components parallel to normal. +ve = entity 2 -> entity 1
        auto v1Parallel = Vector3D::dot(rb1->velocity,contact.worldNormal);
        auto v2Parallel = Vector3D::dot(rb2->velocity,contact.worldNormal);
        
        //velocity components perpendicular to normal
        auto v1Perp = rb1->velocity - (contact.worldNormal * v1Parallel);
        auto v2Perp = rb2->velocity - (contact.worldNormal * v2Parallel);

        if(showDebug){
            std::cout << "v1Parallel = " << v1Parallel << " v1Perp = " << v1Perp << std::endl;
            std::cout << "v2Parallel = " << v2Parallel << " v2Perp = " << v2Perp <<  std::endl;
        }
        
        auto preVParallel = v1Parallel + v2Parallel;
        auto restitution = 0.5f * (rb1->elasticity + rb2->elasticity);
        auto preMomentum = preVParallel * (rb1->mass + rb2->mass);
  
        auto impulse = ((restitution + 1) * (v2Parallel - v1Parallel))/(rb1->invMass + rb2->invMass);
        
        auto newV1Parallel = v1Parallel + (impulse/rb1->mass);
        auto newV2Parallel = v2Parallel - (impulse/rb2->mass);

        auto postMomentum = (newV1Parallel + newV2Parallel)* (rb1->mass + rb2->mass);

        if(showDebug){
            std::cout << "New v1Parallel = " << newV1Parallel << std::endl;
            std::cout << "New v2Parallel = " << newV2Parallel << std::endl;
            std::cout << "Pre-Momentum: " << preMomentum << " Post-Momentum: " << postMomentum << std::endl;
        }
        
        rb1->velocity = contact.worldNormal * (float)newV1Parallel + v1Perp;
        rb2->velocity = contact.worldNormal * (float)newV2Parallel + v2Perp;
        
    }
}

void PhysicsSystem::correctPenetrations(World &world)
{
    //TODO
}

std::vector<Contact> PhysicsSystem::detectCollisions(World& world)
{
    
    std::vector<Contact> contacts;
    
    //put all candidate objects into a vector
    std::vector<CollisionCandidate> candidates;

    world.each<Transform,RigidBody>([&](ECS::Handle<ECS::Entity> entity, Transform& transform, RigidBody& rigidbody)
        {candidates.emplace_back(CollisionCandidate(entity,&rigidbody,&transform));});

    //iterate over pairs of candidates
    
    for(std::size_t i = 0; i < candidates.size()-1; i++)
        for(std::size_t j = i+1; j < candidates.size(); j++)
        {
            auto& candidate1 = candidates[i];
            auto& candidate2 = candidates[j];
            
            //sphere to sphere
            if(candidate1.rigidbody->collider_.type_ == ColliderType::Sphere && candidate2.rigidbody->collider_.type_ == ColliderType::Sphere)
            {
                //check if colliding
                auto separationVector = candidate1.transform->position_ - candidate2.transform->position_;
                auto sumColliders = candidate1.rigidbody->collider_.r_ + candidate2.rigidbody->collider_.r_;
                
                if(separationVector.magnitude() < sumColliders + collisionThreshold)
                {
                    //populate contact struct
                    Contact contact;
                    contact.entity1 = candidate1;
                    contact.entity2 = candidate2;
                    contact.penetrationDepth = sumColliders - separationVector.magnitude();
                    contact.worldNormal = separationVector.getNormalized();
                    contact.worldLocation = candidate2.transform->position_ + (contact.worldNormal * candidate2.rigidbody->collider_.r_);
                    contacts.push_back(contact);
                    if(showDebug){
                        contact.worldLocation.print("Collision at world location: ");
                        std::cout << "Entities = " << i << " and " << j << std::endl;
                    }
                }
           
            }
            else
            {
                std::cout << "Collision type not supported yet" << std::endl;
            }
                    
        }

    return contacts;
}


