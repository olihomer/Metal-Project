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
        
        //normal impulse
        
        float relativeNormalSpeed = Vector3D::dot((rb2->velocity - rb1->velocity),contact.worldNormal);
        if(relativeNormalSpeed<=0.0f)continue; //objects are moving apart, so skip
        
        auto momentum0 = rb1->velocity * rb1->mass + rb2->velocity * rb2->mass;
        auto angularMomentum0 = rb1->angularMomentum.magnitude() + rb2->angularMomentum.magnitude();
    
        float restitution = 0.5f * (rb1->elasticity + rb2->elasticity);
        
        auto Jn = contact.worldNormal * (Vector3D::dot((rb2->velocity - rb1->velocity),contact.worldNormal) * (restitution + 1))*(1/(rb1->invMass + rb2->invMass));

        //tangential impulse
        
        auto r1 = contact.worldLocation - contact.entity1.transform->position_; //vector from centre of mass to contact point
        auto r2 = contact.worldLocation - contact.entity2.transform->position_;
        
        auto vContact1 = Vector3Dd(rb1->velocity) + Vector3Dd::cross(rb1->angularVelocity,r1); //velocity at contact point
        auto vContact2 = Vector3Dd(rb2->velocity) + Vector3Dd::cross(rb2->angularVelocity,r2);
        
        auto vRel = vContact2 - vContact1; //relative velocity at contact point
        auto Vt = vRel - contact.worldNormal * Vector3Dd::dot(vRel,contact.worldNormal); //tangential component of relative velocity at contact point
        auto t = Vt.getNormalized(); //direction of tangent
        
        auto r1Xt = Vector3Dd::cross(r1,t);
        auto r2Xt = Vector3Dd::cross(r2,t);
        
        auto oneOverM = rb1->invMass + rb2->invMass + Vector3Dd::dot(r1Xt,rb1->inverseWorldInertia * r1Xt) + Vector3Dd::dot(r2Xt,rb2->inverseWorldInertia * r2Xt);
        auto effectiveMass = 1 / oneOverM; //effective mass
        
        auto mu = 0.5f * (rb1->mu + rb2->mu); //average friction coefficient
        
        auto Jt =  Vt.getNormalized() * -std::min<double>(mu * Jn.magnitude(), effectiveMass * Vt.magnitude()); //tangential impulse
        
        Jn.print("Jn");
        Jt.print("Jt");
        
        auto Jsum = Jn - Vector3D(Jt);  //combine normal and tangential impulse
        
        rb1->velocity = rb1->velocity + (Jsum * rb1->invMass); //update translational velocity
        rb2->velocity = rb2->velocity - (Jsum * rb2->invMass);

        
        auto torque1 = Vector3D::cross(Jt,r1);
        auto torque2 = Vector3D::cross(Jt,r2);
        
        rb1->angularMomentum = rb1->angularMomentum + torque1;
        rb2->angularMomentum = rb2->angularMomentum - torque2;
        
        rb1->angularVelocity = rb1->angularMomentum * rb1->inverseWorldInertia;
        rb2->angularVelocity = rb2->angularMomentum * rb2->inverseWorldInertia;

        
        rb1->velocity.print("NewV1");
        rb2->velocity.print("NewV2");
        
        auto momentum1 = rb1->velocity * rb1->mass + rb2->velocity * rb2->mass;
        auto angularMomentum1 = rb1->angularMomentum.magnitude() + rb2->angularMomentum.magnitude();

        
        if(showDebug ||true){
            Jn.print("impulse3D");
            momentum0.print("Momentum0");
            std::cout << "Mag = " << momentum0.magnitude() << std::endl;
            momentum1.print("Momentum1");
            std::cout << "Mag = " << momentum1.magnitude() << std::endl;
            std::cout << "AngularMomentum0 = " << angularMomentum0 << std::endl;
            std::cout << "AngularMomentum1 = " << angularMomentum1 << std::endl;
        }
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
    
    for(std::size_t i = 0; i + 1 < candidates.size(); i++)
        for(std::size_t j = i + 1; j < candidates.size(); j++)
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


