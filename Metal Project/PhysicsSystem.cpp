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



std::array<bool,6> Cuboid::getOverlappingFaces(Cuboid& cuboid2)
{
    std::array<bool,6> faceRelevant{};
    auto& cuboid1 = *this;
    
    //calculater transform to convert cuboid2 to cuboid1 local coord system
    
    auto R1 = cuboid1.rotation.toMatrix3x3();
    R1.print("R1");
    auto R2 = cuboid2.rotation.toMatrix3x3();
    R2.print("R2");

    auto R2inR1 = R2 * R1.getInverse();
    
    R2inR1.print("R2inR1");

    //calculate vector from cuboid1 to cuboid2 - make sure it is in local coordinates!
    auto pR1 = (cuboid2.p - cuboid1.p) * R1.getInverse();
        
    //transform cuboid2 half lengths
    auto halfLengthsRotated = cuboid2.halfEdge * R2inR1.abs(); //max extents vector relative to R1 origin
        
    //calculate max/min extent vectors for cuboid2
    auto maxCuboid2 = pR1 + halfLengthsRotated;
    auto minCuboid2 = pR1 - halfLengthsRotated;
    
    maxCuboid2.print("Max c2");
    minCuboid2.print("Min c2");

    //calculate max/min extent vectors for cuboid1
    
    auto maxCuboid1 = cuboid1.halfEdge;
    auto minCuboid1 = cuboid1.halfEdge * -1.0f;
    
    maxCuboid1.print("Max c1");
    minCuboid1.print("Min c1");
    
    //check for axis clearance which means they are definitely not colliding
    
    bool axisOverlap = true;
    
    if(maxCuboid2.x_ < minCuboid1.x_ || minCuboid2.x_ > maxCuboid1.x_){std::cout << "X-Axis Clearance\n";axisOverlap=false;}
    if(maxCuboid2.y_ < minCuboid1.y_ || minCuboid2.y_ > maxCuboid1.y_){std::cout << "Y-Axis Clearance\n";axisOverlap=false;}
    if(maxCuboid2.z_ < minCuboid1.z_ || minCuboid2.z_ > maxCuboid1.z_){std::cout << "Z-Axis Clearance\n";axisOverlap=false;}
    
    if(axisOverlap == false)return faceRelevant; // return with an easy false
    
    // All axes are overlapping => need to start looking for separating planes
    
    //Figure out nature of each overlap
        
    faceRelevant[0] = (minCuboid1.x_ > minCuboid2.x_); //face left
    faceRelevant[1] = (maxCuboid1.x_ < maxCuboid2.x_); //face right
    if(faceRelevant[0] && faceRelevant[1]){faceRelevant[0]=false;faceRelevant[1]=false;} //don't know if this is valid, but i'm trying it
    
    faceRelevant[2] = (minCuboid1.y_ > minCuboid2.y_); //face bottom
    faceRelevant[3] = (maxCuboid1.y_ < maxCuboid2.y_); //face top
    if(faceRelevant[2] && faceRelevant[3]){faceRelevant[2]=false;faceRelevant[3]=false;} //don't know if this is valid, but i'm trying it
    
    faceRelevant[4] = (minCuboid1.z_ > minCuboid2.z_); //face backward
    faceRelevant[5] = (maxCuboid1.z_ < maxCuboid2.z_); //face forward
    if(faceRelevant[4] && faceRelevant[5]){faceRelevant[4]=false;faceRelevant[5]=false;} //don't know if this is valid, but i'm trying it
    
    std::cout<<"Relevant faces:" << std::endl;
    for(int i=0;i<6;i++)
    {
        if(faceRelevant[i]){std::cout<<faceNames[i]<<std::endl;}
    }
    
    return faceRelevant;
}


bool Cuboid::isColliding(Cuboid &cuboid2)
{
    Cuboid& cuboid1 = *this;
    
    auto faces1 = cuboid1.getOverlappingFaces(cuboid2);
    if(std::find(faces1.begin(),faces1.end(),true)==faces1.end())return false; //axis separation => no collision
    
    auto faces2 = cuboid2.getOverlappingFaces(cuboid1);
    if(std::find(faces2.begin(),faces2.end(),true)==faces1.end())return false; //axis separation => no collision
    
    //no axis separation, so consider faces + edges
    
    int faceCount1{};
    int faceCount2{};
    
    std::cout << "Relevant faces on cuboid1:" << std::endl;
    for(int i=0;i<6;i++)if(faces1[i]){faceCount1++;std::cout << faceNames[i] << std::endl;}
    
    std::cout << "Relevant faces on cuboid2:" << std::endl;
    for(int i=0;i<6;i++)if(faces2[i]){faceCount2++;std::cout << faceNames[i] << std::endl;}
    
    //Construct some candidate edges
    
    Vector3D edgeComponents[6] = {  Vector3D(-cuboid1.halfEdge.x_,0,0),Vector3D(cuboid1.halfEdge.x_,0,0),
        Vector3D(0,-cuboid1.halfEdge.y_,0),Vector3D(0,cuboid1.halfEdge.y_,0),
        Vector3D(0,0,-cuboid1.halfEdge.z_),Vector3D(0,0,cuboid1.halfEdge.z_)};
    
    //Cuboid 1:
    
    int edgeCount = 0;
    
    std::vector<std::pair<Vector3D,Vector3D>> edges1;
    
    std::cout << "Cuboid 1 candidate edges:" << std::endl;
    
    for(int index = 0; index < 6; index++)
    {
        if(edgeCount==faceCount1-1){break;}

        if(faces1[index])
        {
            for(int j=index+1;j<6;j++)
            {
                if(faces1[j])
                {
                    //now have a pair, so construct the edge
                    Vector3D v1 = edgeComponents[index] + edgeComponents[j];
                    Vector3D v2 = v1;
                    //there will be a zero component. set it to the +/- half length of the missing component
                    if(v1.x_ == 0){v1.x_=-cuboid1.halfEdge.x_;v2.x_=-v1.x_;}
                    if(v1.y_ == 0){v1.y_=-cuboid1.halfEdge.y_;v2.y_=-v1.y_;}
                    if(v1.z_ == 0){v1.z_=-cuboid1.halfEdge.z_;v2.z_=-v1.z_;}
                    
                    edges1.push_back(std::pair(v1,v2));
                    v1.print("v1");
                    v2.print("v2");
                    
                    if(++edgeCount==faceCount1-1)break;
                }
            }
        }
    }
    
    //Cuboid 2:
    
    Vector3D edgeComponents2[6] = {  Vector3D(-cuboid2.halfEdge.x_,0,0),Vector3D(cuboid2.halfEdge.x_,0,0),
        Vector3D(0,-cuboid2.halfEdge.y_,0),Vector3D(0,cuboid2.halfEdge.y_,0),
        Vector3D(0,0,-cuboid2.halfEdge.z_),Vector3D(0,0,cuboid2.halfEdge.z_)};
    
    edgeCount = 0;
    
    std::vector<std::pair<Vector3D,Vector3D>> edges2;
    std::cout << "Cuboid 2 candidate edges:" << std::endl;

    
    for(int index = 0; index < 6; index++)
    {
        if(edgeCount==faceCount2-1)break;
        if(faces2[index])
        {
            for(int j=index+1;j<6;j++)
            {
                if(faces2[j])
                {
                    //now have a pair, so construct the edge
                    Vector3D v1 = edgeComponents2[index] + edgeComponents2[j];
                    Vector3D v2 = v1;
                    //there will be a zero component. set it to the +/- half length of the missing component
                    if(v1.x_ == 0){v1.x_=-cuboid2.halfEdge.x_;v2.x_=-v1.x_;}
                    if(v1.y_ == 0){v1.y_=-cuboid2.halfEdge.y_;v2.y_=-v1.y_;}
                    if(v1.z_ == 0){v1.z_=-cuboid2.halfEdge.z_;v2.z_=-v1.z_;}
                    
                    edges2.push_back(std::pair(v1,v2));
                    v1.print("v1");
                    v2.print("v2");
                    
                    if(++edgeCount==faceCount2-1)break;
                }
            }
        }
    }
    
    //recreate change of coordinate stuff so that we can work in R1 coordinates for everything
    
    auto R1 = cuboid1.rotation.toMatrix3x3();
    auto R2 = cuboid2.rotation.toMatrix3x3();
    
    auto R2inR1 = R2 * R1.getInverse();
    auto R1inR2 = R1 * R2.getInverse();

    //create normals with pairs of edges (in cuboid 1 coords)
    
    for(auto edge1: edges1)
    {
        for(auto edge2: edges2)
        {
            //First test Cuboid 1 in local coordinates
            auto vector1 = edge1.second-edge1.first;
            auto vector2 = (edge2.second-edge2.first) * R2inR1;
            auto normal = Vector3D::cross(vector1, vector2);
            normal.normalize();
            normal.print("normal 1");
   
            //test to see if that plane separates the objects
            auto r =    cuboid1.halfEdge.x_*abs(Vector3D::dot(Vector3D(1,0,0),normal)) +
                        cuboid1.halfEdge.y_*abs(Vector3D::dot(Vector3D(0,1,0),normal)) +
                        cuboid1.halfEdge.z_*abs(Vector3D::dot(Vector3D(0,0,1),normal));
            
            auto c = cuboid1.p * R1.getInverse();
            
            auto projMin = Vector3D::dot(c,normal)-r;
            auto projMax = Vector3D::dot(c,normal)+r;
            bool goodPlane1 = false;
            if((projMin<0 && projMax<0) || (projMin>0 && projMax>0)){goodPlane1 = true;std::cout<<"Plane works on Cuboid 1"<<std::endl;}
            
            std::cout << "ProjMin " << projMin << " ProjMax " << projMax << std::endl;
            
            //Next test Cuboid 2 in local coordinates
            vector1 = (edge1.second-edge1.first) * R1inR2;
            vector2 = (edge2.second-edge2.first);
            normal = Vector3D::cross(vector1, vector2);
            normal.normalize();
            normal.print("normal 2");
            
            //test to see if that plane separates the objects
            r =    cuboid2.halfEdge.x_*abs(Vector3D::dot(Vector3D(1,0,0),normal)) +
                        cuboid2.halfEdge.y_*abs(Vector3D::dot(Vector3D(0,1,0),normal)) +
                        cuboid2.halfEdge.z_*abs(Vector3D::dot(Vector3D(0,0,1),normal));
            
            c = cuboid2.p * R2.getInverse();
            
            projMin = Vector3D::dot(c,normal)-r;
            projMax = Vector3D::dot(c,normal)+r;
            
            bool goodPlane2 = false;
            if((projMin<0 && projMax<0) || (projMin>0 && projMax>0)){goodPlane2 = true;std::cout<<"Plane works on Cuboid 2"<<std::endl;}
            
            std::cout << "ProjMin " << projMin << " ProjMax " << projMax << std::endl;
            
            if(goodPlane1 && goodPlane2) return false; //both planes seem to separate the objects => no collision
            
        }
    }
    
    //Run out of ideas => must be colliding
    
    return true;
}

