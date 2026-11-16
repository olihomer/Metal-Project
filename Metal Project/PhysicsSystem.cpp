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
    
    integratePositions(world, dt);

    auto contacts = detectCollisions(world);
    
    solveContacts(world, contacts, dt);
    
    correctPenetrations(world, contacts);

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
    if(rigidbody.collider_.type_==ColliderType::GroundPlane)return;

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
    if(rigidbody.collider_.type_==ColliderType::GroundPlane)return;
    //accumulate and apply forces
    ForceAccumulator forces = accumulateForces(transform, rigidbody, dt);
    
    Vector3D acceleration = forces.netForce * (1.0f/rigidbody.mass);
        
    //linear velocity
    rigidbody.velocity = rigidbody.velocity + acceleration * dt;
    
    //angular velocity
    rigidbody.updateWorldInertia(transform.rotation_);

    rigidbody.angularMomentum += forces.netTorque * dt;
    rigidbody.angularVelocity = rigidbody.angularMomentum * rigidbody.inverseWorldInertia;
    
    applyDamping(rigidbody);
    
}


ForceAccumulator PhysicsSystem::accumulateForces(const Transform& transform, RigidBody& rigidbody, double dt) const
{
    ForceAccumulator forces;
    if(rigidbody.collider_.type_==ColliderType::GroundPlane)return forces;

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
        
        rb1->velocity.print("rb1 v0");
        rb2->velocity.print("rb2 v0");
        
        auto totalL0 =
            Vector3Dd::cross(
                Vector3Dd(contact.entity1.transform->position_),
                Vector3Dd(rb1->velocity) * rb1->mass
            )
            + rb1->angularMomentum
            + Vector3Dd::cross(
                Vector3Dd(contact.entity2.transform->position_),
                Vector3Dd(rb2->velocity) * rb2->mass
            )
            + rb2->angularMomentum;
        
        auto momentum0 = rb1->velocity * rb1->mass + rb2->velocity * rb2->mass;
        auto energy0 = pow(rb1->velocity.magnitude(),2) * 0.5 * rb1->mass + pow(rb2->velocity.magnitude(),2) * 0.5 * rb2->mass + Vector3Dd::dot(rb1->angularMomentum,rb1->angularVelocity) * 0.5 + Vector3Dd::dot(rb2->angularMomentum,rb2->angularVelocity) * 0.5;
    
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
        
        //spinning friction
        
        auto relativeSpinSpeed = Vector3Dd::dot((rb2->angularVelocity - rb1->angularVelocity),contact.worldNormal);
        auto effectiveInertia = Vector3Dd::dot(Vector3Dd(contact.worldNormal) * (rb1->inverseWorldInertia + rb2->inverseWorldInertia), Vector3Dd(contact.worldNormal));
        auto Js = -relativeSpinSpeed/effectiveInertia;
        
        float muSpin = 0.8f;
        float effectiveArea = 0.02f;
        auto maxSpin = Jn.magnitude() * muSpin * effectiveArea;
        
        auto magJS = std::clamp<double>(Js,-maxSpin,maxSpin);
        
        auto spinImpulse = contact.worldNormal * magJS;
        
        //rolling friction
        Vector3Dd rollingImpulse{};
        
        if(contact.isRolling)
        {
            //is a sphere-ground collision. rb2 = the sphere
            auto rollingW = rb2->angularVelocity - contact.worldNormal * Vector3Dd::dot(rb2->angularVelocity,contact.worldNormal);
            auto Jr = rb2->worldInertia * rollingW * -1;
            auto rollingMag = Jr.magnitude();
            std::cout << "ROLLING MAG " << rollingMag << std::endl;
            float muRoll = 0.02f;
            auto maxRollMag = Jn.magnitude() * muRoll;
            std::cout << "MAX MAG " << maxRollMag << std::endl;
            rollingMag = std::clamp<double>(rollingMag,-maxRollMag,maxRollMag);
            std::cout << "clamped " << rollingMag << std::endl;
            
            rollingImpulse = Jr.getNormalized() * rollingMag;
            rollingImpulse.print("Roll impulse");
        }
                
        //combine impulses
        
        Jn.print("Jn");
        Jt.print("Jt");
        
        auto Jsum = Jn - Vector3D(Jt);  //combine normal and tangential impulse
        
        rb1->velocity = rb1->velocity + (Jsum * rb1->invMass); //update translational velocity
        rb2->velocity = rb2->velocity - (Jsum * rb2->invMass);

        std::cout << "Tangential speed: " << Vt.magnitude() << '\n';
        std::cout << "Tangential effective mass: " << effectiveMass << '\n';
        std::cout << "Friction impulse magnitude: " << Jt.magnitude() << '\n';
        
        auto torque1 = Vector3D::cross(Jt,r1); //calculate torques from tangential impulse and radius
        auto torque2 = Vector3D::cross(Jt,r2);
        
        rb1->angularMomentum = rb1->angularMomentum + torque1 - spinImpulse - rollingImpulse; //apply torques to angular momentum
        rb2->angularMomentum = rb2->angularMomentum - torque2 + spinImpulse + rollingImpulse;
        
        rb1->angularVelocity = rb1->angularMomentum * rb1->inverseWorldInertia; //apply angular momentum to angular velocity
        rb2->angularVelocity = rb2->angularMomentum * rb2->inverseWorldInertia;
        
        rb1->velocity.print("NewV1");
        rb2->velocity.print("NewV2");
 
        auto momentum1 = rb1->velocity * rb1->mass + rb2->velocity * rb2->mass;
        auto energy1 = pow(rb1->velocity.magnitude(),2) * 0.5 * rb1->mass + pow(rb2->velocity.magnitude(),2) * 0.5 * rb2->mass + Vector3Dd::dot(rb1->angularMomentum,rb1->angularVelocity) * 0.5 + Vector3Dd::dot(rb2->angularMomentum,rb2->angularVelocity) * 0.5;
        
        auto totalL1 =
            Vector3Dd::cross(
                Vector3Dd(contact.entity1.transform->position_),
                Vector3Dd(rb1->velocity) * rb1->mass
            )
            + rb1->angularMomentum
            + Vector3Dd::cross(
                Vector3Dd(contact.entity2.transform->position_),
                Vector3Dd(rb2->velocity) * rb2->mass
            )
            + rb2->angularMomentum;
        
        if(showDebug ||true){
            totalL0.print("Angular Momentum0");
            std::cout << "Mag = " << totalL0.magnitude() << std::endl;
            totalL1.print("Angular Momentum1");
            std::cout << "Mag = " << totalL1.magnitude() << std::endl;
            momentum0.print("Momentum0");
            std::cout << "Mag = " << momentum0.magnitude() << std::endl;
            momentum1.print("Momentum1");
            std::cout << "Mag = " << momentum1.magnitude() << std::endl;
            std::cout << "Energy0 = " << energy0 << std::endl;
            std::cout << "Energy1 = " << energy1 << std::endl;
        }
    }
}

void PhysicsSystem::correctPenetrations(World &world, const std::vector<Contact>& contacts)
{
    //iterate through contacts
    for(auto contact: contacts)
    {
        std::cout << "Penetration: " << contact.penetrationDepth << std::endl;
        auto* rb1 = contact.entity1.rigidbody;
        auto* rb2 = contact.entity2.rigidbody;
        auto* trans1 = contact.entity1.transform;
        auto* trans2 = contact.entity2.transform;
        
        auto correctionVector1 = (contact.worldNormal * contact.penetrationDepth) * ((rb1->invMass)/(rb1->invMass+rb2->invMass));
        auto correctionVector2 = (contact.worldNormal * -contact.penetrationDepth) * ((rb2->invMass)/(rb1->invMass+rb2->invMass));

        trans1->position_ = trans1->position_ + correctionVector1;
        trans2->position_ = trans2->position_ + correctionVector2;
    }
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
            auto type1 = candidate1.rigidbody->collider_.type_;
            auto type2 = candidate2.rigidbody->collider_.type_;
            
            
            //sphere to sphere
            if(type1 == ColliderType::Sphere && type2 == ColliderType::Sphere)
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
                        contact.worldLocation.print("Sphere-Sphere collision at world location: ");
                        std::cout << "Entities = " << i << " and " << j << std::endl;
                    }
                }
           
            }
            else if((type1 == ColliderType::Sphere && type2 == ColliderType::GroundPlane) || (type1 == ColliderType::GroundPlane && type2 == ColliderType::Sphere))
            {
                CollisionCandidate& ground = (type1 == ColliderType::Sphere) ? candidate2 : candidate1;
                CollisionCandidate& sphere = (type2 == ColliderType::Sphere) ? candidate2 : candidate1;

                auto separation = (sphere.transform->position_.y_ - sphere.rigidbody->collider_.r_) - ground.transform->position_.y_;

                if(separation<0)
                {
                    //populate contact struct
                    Contact contact;
                    contact.entity1 = ground;
                    contact.entity2 = sphere;
                    contact.penetrationDepth = -separation;
                    contact.worldNormal = Vector3D(0,-1,0);
                    contact.worldLocation = Vector3D(sphere.transform->position_.x_,ground.transform->position_.y_,sphere.transform->position_.z_);
                    contact.isRolling = true;
                    contacts.push_back(contact);
                    ground.rigidbody->invMass = 0;
                    ground.rigidbody->inverseBodyInertia = Matrix3x3d() * 0;
                    ground.rigidbody->inverseWorldInertia = Matrix3x3d() * 0;
                    if(showDebug){
                        contact.worldLocation.print("Sphere-ground collision at world location: ");
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


