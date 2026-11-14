//
//  RigidBody.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef RigidBody_hpp
#define RigidBody_hpp

#include "Pool.hpp"
#include "Vector3D.h"
#include "Matrix3x3d.hpp"
#include "Collider.hpp"
#include "Mesh.hpp"
#include "Quaternion.h"

class RigidBody
{
public:
    RigidBody(){;};
    RigidBody(MeshAsset::physics physics, double density = 800)
    {
        inverseBodyInertia = physics.inverseBodyInertia_ * (1/density);
        bodyInertia = physics.bodyInertia_ * density;
        mass = physics.massAtUnitDensity_ * density;
        std::cout << "Creating rigid body with mass: " << mass << std::endl;
        bodyInertia.print("Body inertia");
        inverseBodyInertia.print("Inverse body inertia");
        invMass = 1/mass;
    }

    double mass = 1.0;
    double invMass = 1.0;
    Matrix3x3d bodyInertia;
    Matrix3x3d worldInertia;
    Matrix3x3d inverseBodyInertia;
    Matrix3x3d inverseWorldInertia;
    Vector3D velocity = Vector3D(0.0,0.0,0.0);
    Vector3Dd angularVelocity;
    Vector3Dd angularMomentum = Vector3Dd(0.0,0.0,0.0);
    float dragCoefficient = 0.5f;
    float crossSectionArea = 0.1f;
    float mu = 1.0f;
    float elasticity = 0.6f;
    void setCollider(ColliderType type, float r = 0, float x = 0, float y = 0, float z = 0){collider_=  Collider(type,r,x,y,z);}
    Collider collider_{ColliderType::None};
    void updateWorldInertia(Quaternion rotation);
    
private:
};

#endif /* RigidBody_hpp */
