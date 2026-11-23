//
//  PhysicsSystem.cpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#include "PhysicsSystem.hpp"
#include "Matrix2x2.hpp"

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



bool Cuboid::isColliding(Cuboid &cuboid2)
{
    Cuboid& cuboid1 = *this;
    bool colliding = true;
    float minPenetration = MAXFLOAT;
    Vector3D minPenetrationNormal{};
    int minPenetrationIndex{};
    
    //Candidate edges are just the half edge vectors, stored here in coordinates local to each cuboid
    
    std::array<Vector3D,3> edges1 = {Vector3D(cuboid1.halfEdge.x_,0,0),Vector3D(0,cuboid1.halfEdge.y_,0),Vector3D(0,0,cuboid1.halfEdge.z_)};
    std::array<Vector3D,3> edges2 = {Vector3D(cuboid2.halfEdge.x_,0,0),Vector3D(0,cuboid2.halfEdge.y_,0),Vector3D(0,0,cuboid2.halfEdge.z_)};
    std::array<std::string,15> descriptions = {"Cuboid 1 x face", "Cuboid 1 y face","Cuboid 1 z face","Cuboid 2 x face", "Cuboid 2 y face","Cuboid 2 z face","Edge cross xx","Edge cross xy","Edge cross xz","Edge cross yx","Edge cross yy","Edge cross yz","Edge cross zx","Edge cross zy","Edge cross zz"};

    
    auto R1 = cuboid1.rotation.toMatrix3x3();
    auto R2 = cuboid2.rotation.toMatrix3x3();
    auto R2inR1 = R2 * R1.getInverse();
    auto R1inR2 = R1 * R2.getInverse();

    auto p1 = cuboid1.p * R1.getInverse();
    auto p2 = cuboid2.p * R1.getInverse();
    Vector3D p1p2 = p2-p1; // vector from p1 to p2 in cuboid 1 coordinates
    
    std::array<Vector3D,15> normals;
    
    for(int i=0;i<3;i++)
    {
        normals[i] = edges1[i].getNormalized();
        normals[i+3] = (edges2[i] * R2inR1).getNormalized();
        for(int j=0;j<3;j++)
            normals[6+(i*3)+j]=(Vector3D::cross(edges1[i],edges2[j] * R2inR1)); //normalise later: need to check magnitude
    }
    
    for(int i=0; i<15;i++)
    {
        auto& normal = normals[i];
        if(normal.magnitude()<0.0001f)continue;
        normal.normalize();
        
        //calculate half-extents along the normal
        
        auto r1 =    cuboid1.halfEdge.x_*abs(Vector3D::dot(Vector3D(1,0,0),normal)) +
        cuboid1.halfEdge.y_*abs(Vector3D::dot(Vector3D(0,1,0),normal)) +
        cuboid1.halfEdge.z_*abs(Vector3D::dot(Vector3D(0,0,1),normal));
        
        auto r2 =    cuboid2.halfEdge.x_*abs(Vector3D::dot(Vector3D(1,0,0),normal*R1inR2)) +
        cuboid2.halfEdge.y_*abs(Vector3D::dot(Vector3D(0,1,0),normal*R1inR2)) +
        cuboid2.halfEdge.z_*abs(Vector3D::dot(Vector3D(0,0,1),normal*R1inR2));
    
        //calculate intervals along the normal
        
        auto iMin1 = -r1;
        auto iMax1 = r1;
        auto iMin2 = Vector3D::dot((p2-p1),normal) - r2;
        auto iMax2 = Vector3D::dot((p2-p1),normal) + r2;
        
        std::cout << "Intervals: " << iMin1 << "," << iMax1 << "  " << iMin2 << "," << iMax2 << std::endl;
        
        if(iMax2<iMin1 || iMin2>iMax1){
            std::cout << "Normal index " << i << std::endl;
            std::cout<<"Separation = " << std::max(iMin1-iMax2,iMin2-iMax1) << std::endl;
            colliding = false;
        }
        else
        {
            float penetration = std::min(iMax1-iMin2,iMax2-iMin1);
            if(penetration < minPenetration)
            {
                minPenetration = penetration;
                minPenetrationIndex = i;
                minPenetrationNormal=normals[i];
                if(Vector3D::dot(minPenetrationNormal,p1p2)<0){minPenetrationNormal = minPenetrationNormal * -1.0f;descriptions[i] += "(negative)";}
            }
            
            std::cout << "Normal index " << i << std::endl;
            std::cout << "Penetration = " << penetration << "\n" << std::endl;
            
        }
    
    }
    
    if(colliding)
    {
        std::cout << "Minimum penetration = " << minPenetration << " at normal " << minPenetrationNormal << " (index = " << minPenetrationIndex << ")" << std::endl;
        std::cout << "Normal corresponds to " << descriptions[minPenetrationIndex] << std::endl;
        std::cout << "Normal in cuboid 2 coordinates = " << minPenetrationNormal * R1inR2 << std::endl;
    
        Vector3D va1{},vb1{},edge1{};
        Vector3D va2{},vb2{},edge2{};
        
        if(minPenetrationIndex>5) //edge-to-edge contact
        {
            //construct the cuboid 1 edge
            if(minPenetrationIndex/3<3) //x-axis edge
            {
                va1.x_ = 1.0f;
                va1.y_ = (minPenetrationNormal.y_ < 0) ? -1.0f : 1.0f;
                va1.z_ = (minPenetrationNormal.z_ < 0) ? -1.0f : 1.0f;
                va1 = Vector3D(va1.x_*cuboid1.halfEdge.x_,va1.y_*cuboid1.halfEdge.y_,va1.z_*cuboid1.halfEdge.z_);
                vb1 = Vector3D(-va1.x_,va1.y_,va1.z_);
            }
            else if(minPenetrationIndex/3<4) //y-axis edge
            {
                va1.y_ = 1.0f;
                va1.x_ = (minPenetrationNormal.x_ < 0) ? -1.0f : 1.0f;
                va1.z_ = (minPenetrationNormal.z_ < 0) ? -1.0f : 1.0f;
                va1 = Vector3D(va1.x_*cuboid1.halfEdge.x_,va1.y_*cuboid1.halfEdge.y_,va1.z_*cuboid1.halfEdge.z_);
                vb1 = Vector3D(va1.x_,-va1.y_,va1.z_);
            }
            else //z-axis edge
            {
                va1.z_ = 1.0f;
                va1.x_ = (minPenetrationNormal.x_ < 0) ? -1.0f : 1.0f;
                va1.y_ = (minPenetrationNormal.y_ < 0) ? -1.0f : 1.0f;
                va1 = Vector3D(va1.x_*cuboid1.halfEdge.x_,va1.y_*cuboid1.halfEdge.y_,va1.z_*cuboid1.halfEdge.z_);
                vb1 = Vector3D(va1.x_,va1.y_,-va1.z_);
            }

            std::cout << "Cuboid 1 edge = " << va1 << " to " << vb1 << std::endl;
            
            //construct the cuboid 2 edge
            auto minPenetrationNormal2 = minPenetrationNormal * R1inR2 * -1.0f;
            if(minPenetrationIndex%3==0) //x-axis edge
            {
                va2.x_ = 1.0f;
                va2.y_ = (minPenetrationNormal2.y_ < 0) ? -1.0f : 1.0f;
                va2.z_ = (minPenetrationNormal2.z_ < 0) ? -1.0f : 1.0f;
                va2 = Vector3D(va2.x_*cuboid1.halfEdge.x_,va2.y_*cuboid1.halfEdge.y_,va2.z_*cuboid1.halfEdge.z_);
                vb2 = Vector3D(-va2.x_,va2.y_,va2.z_);
            }
            else if(minPenetrationIndex%3==1) //y-axis edge
            {
                va2.y_ = 1.0f;
                va2.x_ = (minPenetrationNormal2.x_ < 0) ? -1.0f : 1.0f;
                va2.z_ = (minPenetrationNormal2.z_ < 0) ? -1.0f : 1.0f;
                va2 = Vector3D(va2.x_*cuboid1.halfEdge.x_,va2.y_*cuboid1.halfEdge.y_,va2.z_*cuboid1.halfEdge.z_);
                vb2 = Vector3D(va2.x_,-va2.y_,va2.z_);
            }
            else //z-axis edge
            {
                va2.z_ = 1.0f;
                va2.x_ = (minPenetrationNormal2.x_ < 0) ? -1.0f : 1.0f;
                va2.y_ = (minPenetrationNormal2.y_ < 0) ? -1.0f : 1.0f;
                va2 = Vector3D(va2.x_*cuboid2.halfEdge.x_,va2.y_*cuboid2.halfEdge.y_,va2.z_*cuboid2.halfEdge.z_);
                vb2 = Vector3D(va2.x_,va2.y_,-va2.z_);
            }
            std::cout << "Cuboid 2 edge = " << va2 << " to " << vb2 << std::endl;

            //construct simultaneous equation - using cuboid 1 coordinate system
            
            auto d = va1 - (p1p2 + va2 * R2inR1); //vector from cuboid 1 edge to cuboid 2 edge
            auto n1 = (vb1-va1).getNormalized(); // direction of edge1
            auto n2 = ((vb2-va2)*R2inR1).getNormalized(); //direction of edge 2
            auto n1n2 = Vector3D::dot(n1,n2);
            
            Matrix2x2 solver{};
            Vector2D constants(Vector3D::dot(d,n2), Vector3D::dot(d,n1));
            solver.mat_[0][0] = -n1n2;
            solver.mat_[0][1] = -1;
            solver.mat_[1][0] = 1;
            solver.mat_[1][1] = n1n2;
            auto inverse = solver.getInverse();
            auto solution = constants * inverse;
            
            Vector3D solution1 = (n1 * solution.x_) + va1; // cuboid1 collision point in local coords
            Vector3D solution2 = (n2 * solution.y_) + p1p2 + va2*R2inR1; //cuboid2 collision point in cuboid1 local coords
            
            std::cout << "Cuboid 1 local coords - Solution 1: " << solution1 << " Solution 2: " << solution2 << std::endl;
            
            auto solution1World = p1 + solution1 * R1;
            auto solution2World = p1 + solution2 * R1;
            auto midSolutionWorld = (solution1World + solution2World) * 0.5f;
            
            std::cout << "World coords - Solution 1: " << solution1World << " Solution 2: " << solution2World << " Average: " << midSolutionWorld << std::endl;
            
            
        }
    
    
    
    
    }
    
    return colliding;
}

