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

    contacts_ = detectCollisions(world);
        
    if(contacts_.size()>0)solveContactsIterator(world, contacts_, dt, 10);
    
    integratePositions(world, dt);
    
    auto temp = detectCollisions(world);
    
    correctPenetrations(world, temp);

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
    
    acceleration.print("Acceleration");
    transform.position_.print("Location");
    
    //linear velocity
    rigidbody.velocity = rigidbody.velocity + acceleration * dt;
    
    //angular velocity
    rigidbody.updateWorldInertia(transform.rotation_);

    rigidbody.angularMomentum += forces.netTorque * dt;
    rigidbody.angularVelocity = rigidbody.angularMomentum * rigidbody.inverseWorldInertia;
    
    //applyDamping(rigidbody);
    
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

void PhysicsSystem::correctPenetrations(World &world, std::vector<Contact>& contacts) const
{
    //iterate through contacts
    for(auto& contact: contacts)
    {
        if(showDebug) std::cout << "Penetration: " << contact.penetrationDepth << std::endl;
        auto* rb1 = contact.entity1.rigidbody;
        auto* rb2 = contact.entity2.rigidbody;
        auto* trans1 = contact.entity1.transform;
        auto* trans2 = contact.entity2.transform;
        
        trans2->position_.print("Position 2");
        
        auto correctionVector1 = (contact.worldNormal * contact.penetrationDepth) * ((rb1->invMass)/(rb1->invMass+rb2->invMass));
        auto correctionVector2 = (contact.worldNormal * -contact.penetrationDepth) * ((rb2->invMass)/(rb1->invMass+rb2->invMass));

        trans1->position_ = trans1->position_ + correctionVector1;
        trans2->position_ = trans2->position_ + correctionVector2;
        
        correctionVector1.print("Correction 1");
        correctionVector2.print("Correction 2");
        
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
                    contact.penetrationDepth = std::max(0.0f,sumColliders - separationVector.magnitude());
                    contact.worldNormal = separationVector.getNormalized();
                    
                    //store the target normal separation velocity
                    double vn = Vector3D::dot(candidate2.rigidbody->velocity-candidate1.rigidbody->velocity,contact.worldNormal);
                    double e = 0.5 * (candidate1.rigidbody->elasticity + candidate2.rigidbody->elasticity);
                    contact.targetNormalVelocity = vn > bounceThreshold ? -e * vn: 0.0;

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

                //separation will be negative if penetration exists
                
                if(separation < collisionThreshold) //also captures objects that are close but not penetrating
                {
                    //populate contact struct
                    Contact contact;
                    contact.entity1 = ground;
                    contact.entity2 = sphere;
                    contact.penetrationDepth = std::max(-separation,0.0f); //penetration depth is a positive number (or zero for no penetration)
                    contact.worldNormal = Vector3D(0,-1,0);
                    
                    //store the target normal separation velocity
                    double vn = Vector3D::dot(sphere.rigidbody->velocity,contact.worldNormal);
                    double e = 0.5 * (sphere.rigidbody->elasticity + ground.rigidbody->elasticity);
                    contact.targetNormalVelocity = vn > bounceThreshold ? -e * vn: 0.0;

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

void PhysicsSystem::solveContactsIterator(World &world, std::vector<Contact>& contacts, double dt, int iterations)
{
    Vector3Dd totalL0,momentum0;
    double energy0;
    
    for(int i = 0; i < iterations; i++)
    {
        std::cout << "Iteration " << i << " of " << iterations << ". Number of contacts = " << contacts.size() << std::endl;
        
        //iterate through contacts
        for(auto& contact: contacts)
        {
            auto rb1=contact.entity1.rigidbody;
            auto rb2=contact.entity2.rigidbody;
            rb2->velocity.print("rb2 v0");
            //normal linear impulse
            
            float relativeNormalSpeed = Vector3D::dot((rb2->velocity - rb1->velocity),contact.worldNormal);
            //if(relativeNormalSpeed < 0)continue; //objects are moving apart, so skip
            
            if(showDebug){
                rb1->velocity.print("rb1 v0");
                rb2->velocity.print("rb2 v0");
                
                totalL0 =
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
                
                momentum0 = rb1->velocity * rb1->mass + rb2->velocity * rb2->mass;
                energy0 = pow(rb1->velocity.magnitude(),2) * 0.5 * rb1->mass + pow(rb2->velocity.magnitude(),2) * 0.5 * rb2->mass + Vector3Dd::dot(rb1->angularMomentum,rb1->angularVelocity) * 0.5 + Vector3Dd::dot(rb2->angularMomentum,rb2->angularVelocity) * 0.5;
            }
                        
            auto deltaJn = ((relativeNormalSpeed - contact.targetNormalVelocity) / (rb1->invMass + rb2->invMass)); //normal impulse
            
            auto& accumulatedJn = contact.impulseAccumulator.normalLinear;
            double oldJn = accumulatedJn;
            
            accumulatedJn = std::max(0.0, oldJn + deltaJn);
            deltaJn = accumulatedJn - oldJn;
            
            auto Jn = contact.worldNormal * deltaJn;
            std::cout << "Jn mag = " << Jn.magnitude()/contact.worldNormal.magnitude() << std::endl;
            
            rb1->velocity = rb1->velocity + (Jn * rb1->invMass); //update translational velocity
            rb2->velocity = rb2->velocity - (Jn * rb2->invMass);

            //tangential linear impulse
            
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
            
            auto& accumulatedJt = contact.impulseAccumulator.tangentialLinear;
            Vector3Dd oldJt = accumulatedJt;
            
            Vector3Dd deltaJt = t * -effectiveMass * Vt.magnitude();
            Vector3Dd newJt = oldJt + deltaJt;
            
            double maxFriction = mu * contact.impulseAccumulator.normalLinear;
            
            if(newJt.magnitude()> maxFriction) newJt = newJt.getNormalized() * maxFriction;
            Vector3Dd Jt = newJt - oldJt;
            
            accumulatedJt = newJt;
            
            rb1->velocity = rb1->velocity - (Jt * rb1->invMass); //update translational velocity
            rb2->velocity = rb2->velocity + (Jt * rb2->invMass);

            //spinning friction -> angular impulse
            
            auto relativeSpinSpeed = Vector3Dd::dot((rb2->angularVelocity - rb1->angularVelocity),contact.worldNormal);
            auto effectiveInverseInertia = Vector3Dd::dot(Vector3Dd(contact.worldNormal) * (rb1->inverseWorldInertia + rb2->inverseWorldInertia), Vector3Dd(contact.worldNormal));
            
            auto Js = -relativeSpinSpeed/effectiveInverseInertia;
            
            float muSpin = 0.5f * (rb1->muSpin + rb2->muSpin);
            constexpr float effectiveLength = 0.02f;
            auto maxSpin = contact.impulseAccumulator.normalLinear * muSpin * effectiveLength;
            
            auto& accumulatedJs = contact.impulseAccumulator.spinAngular;
            auto oldJs = accumulatedJs;
            
            accumulatedJs = std::clamp(oldJs + Js, -maxSpin, maxSpin);
            
            double deltaJs = accumulatedJs - oldJs;
      
            auto spinImpulse = contact.worldNormal * deltaJs; //angular impulse
            
            //rolling friction -> angular impulse
            Vector3Dd rollingImpulse{};
            
            if(contact.isRolling)
            {
                //is a sphere-ground collision. rb2 = the sphere
                auto rollingW = rb2->angularVelocity - contact.worldNormal * Vector3Dd::dot(rb2->angularVelocity,contact.worldNormal);
                
                auto& accumulatedJr = contact.impulseAccumulator.rollAngular;
                Vector3Dd oldJr = accumulatedJr;

                Vector3Dd Jr = rb2->worldInertia * rollingW * -1;
                Vector3Dd newJr = oldJr + Jr;
                
                float muRoll = 0.5f * (rb1->muRoll + rb2->muRoll);
                auto maxRoll = contact.impulseAccumulator.normalLinear * muRoll * rb2->collider_.r_;
                
                if(newJr.magnitude() > maxRoll) newJr  = newJr.getNormalized() * maxRoll;
                
                rollingImpulse = newJr - oldJr;
                accumulatedJr = newJr;
                
            }
            
            //combine impulses
            
            std::cout << i << std::endl;
            rb2->velocity.print("rb2 v1");
            
            auto torque1 = Vector3D::cross(Jt,r1); //calculate torques from tangential linear impulse and radius
            auto torque2 = Vector3D::cross(Jt,r2);
            
            rb1->angularMomentum = rb1->angularMomentum + torque1 - spinImpulse - rollingImpulse; //apply torques and angular impulses to angular momentum
            rb2->angularMomentum = rb2->angularMomentum - torque2 + spinImpulse + rollingImpulse;
            
            rb1->angularVelocity = rb1->angularMomentum * rb1->inverseWorldInertia; //apply angular momentum to angular velocity
            rb2->angularVelocity = rb2->angularMomentum * rb2->inverseWorldInertia;
            
            if(showDebug){
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
                
                
                Jn.print("Jn");
                Jt.print("Jt");
                std::cout << "Tangential speed: " << Vt.magnitude() << '\n';
                std::cout << "Tangential effective mass: " << effectiveMass << '\n';
                std::cout << "Friction impulse magnitude: " << Jt.magnitude() << '\n';
                rb1->velocity.print("NewV1");
                rb2->velocity.print("NewV2");
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
}



bool Cuboid::hasOverlappingFaces(Cuboid& cuboid2)
{
    std::array<bool,6> faceRelevant{};
    auto& cuboid1 = *this;
    
    //calculater transform to convert cuboid2 to cuboid1 local coord system
    
    auto R1 = cuboid1.rotation.toMatrix3x3();
    auto R2 = cuboid2.rotation.toMatrix3x3();

    auto R2inR1 = R2 * R1.getInverse();
    
    //calculate vector from cuboid1 to cuboid2 - make sure it is in local coordinates!
    auto pR1 = (cuboid2.p - cuboid1.p) * R1.getInverse();
        
    //transform cuboid2 half lengths
    auto halfLengthsRotated = cuboid2.halfEdge * R2inR1.abs(); //max extents vector relative to R1 origin
        
    //calculate max/min extent vectors for cuboid2
    auto maxCuboid2 = pR1 + halfLengthsRotated;
    auto minCuboid2 = pR1 - halfLengthsRotated;

    //calculate max/min extent vectors for cuboid1
    
    auto maxCuboid1 = cuboid1.halfEdge;
    auto minCuboid1 = cuboid1.halfEdge * -1.0f;

    //check for axis clearance which means they are definitely not colliding
    
    bool axisOverlap = true;
    
    if(maxCuboid2.x_ < minCuboid1.x_ || minCuboid2.x_ > maxCuboid1.x_){std::cout << "X-Axis Clearance\n";axisOverlap=false;}
    if(maxCuboid2.y_ < minCuboid1.y_ || minCuboid2.y_ > maxCuboid1.y_){std::cout << "Y-Axis Clearance\n";axisOverlap=false;}
    if(maxCuboid2.z_ < minCuboid1.z_ || minCuboid2.z_ > maxCuboid1.z_){std::cout << "Z-Axis Clearance\n";axisOverlap=false;}
    
    if(axisOverlap == false)return false; // return with an easy false
        
    //Return relevant faces
        
    faceRelevant[0] = (minCuboid1.x_ > minCuboid2.x_); //face left
    faceRelevant[1] = (maxCuboid1.x_ < maxCuboid2.x_); //face right
    
    faceRelevant[2] = (minCuboid1.y_ > minCuboid2.y_); //face bottom
    faceRelevant[3] = (maxCuboid1.y_ < maxCuboid2.y_); //face top
    
    faceRelevant[4] = (minCuboid1.z_ > minCuboid2.z_); //face backward
    faceRelevant[5] = (maxCuboid1.z_ < maxCuboid2.z_); //face forward
    
    std::cout<<"Relevant faces:" << std::endl;
    for(int i=0;i<6;i++)
    {
        if(faceRelevant[i]){std::cout<<faceNames[i]<<std::endl;}
    }
    
    return true;
}


bool Cuboid::isColliding(Cuboid &cuboid2)
{
    Cuboid& cuboid1 = *this;
    
    if(!cuboid1.hasOverlappingFaces(cuboid2))return false;
    if(!cuboid2.hasOverlappingFaces(cuboid1))return false;
    
    //Candidate edges are just the half edge vectors, stored here in coordinates local to each cuboid
    
    std::array<Vector3D,3> edges1 = {Vector3D(cuboid1.halfEdge.x_,0,0),Vector3D(0,cuboid1.halfEdge.y_,0),Vector3D(0,0,cuboid1.halfEdge.z_)};
    std::array<Vector3D,3> edges2 = {Vector3D(cuboid2.halfEdge.x_,0,0),Vector3D(0,cuboid2.halfEdge.y_,0),Vector3D(0,0,cuboid2.halfEdge.z_)};
    
    auto R1 = cuboid1.rotation.toMatrix3x3();
    auto R2 = cuboid2.rotation.toMatrix3x3();
    auto R2inR1 = R2 * R1.getInverse();

    //create normals with pairs of edges (in cuboid 1 coords)
    
    for(auto edge1: edges1)
    {
        for(auto edge2: edges2)
        {
            //Put everything in Cuboid 1 local coordinates
            
            auto p1 = cuboid1.p * R1.getInverse();
            auto p2 = cuboid2.p * R1.getInverse();
            
            auto vector1 = edge1;
            auto vector2 = edge2 * R2inR1;
            
            auto normal = Vector3D::cross(vector1, vector2);
            normal.normalize();
            normal.print("normal");
            
            //calculate half-extents along the normal
            auto r1 =    cuboid1.halfEdge.x_*abs(Vector3D::dot(Vector3D(1,0,0),normal)) +
            cuboid1.halfEdge.y_*abs(Vector3D::dot(Vector3D(0,1,0),normal)) +
            cuboid1.halfEdge.z_*abs(Vector3D::dot(Vector3D(0,0,1),normal));
            
            auto r2 = (cuboid2.halfEdge * R2inR1).x_ * abs(Vector3D::dot(Vector3D(1,0,0),normal)) +
            (cuboid2.halfEdge * R2inR1).y_*abs(Vector3D::dot(Vector3D(0,1,0),normal)) +
            (cuboid2.halfEdge * R2inR1).z_*abs(Vector3D::dot(Vector3D(0,0,1),normal));
            
            //calculate intervals along the normal
            
            auto iMin1 = -r1;
            auto iMax1 = r1;
            auto iMin2 = Vector3D::dot((p2-p1),normal) - r2;
            auto iMax2 = Vector3D::dot((p2-p1),normal) + r2;
            
            std::cout << "Intervals: " << iMin1 << "," << iMax1 << "  " << iMin2 << "," << iMax2 << std::endl;
            
            if(iMax2<iMin1||iMin2>iMax1){std::cout << "NORMAL WORKS - NO OVERLAP" << std::endl;return false;}
            
        }
    }
    //must be colliding
    return true;
}

