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

class RigidBody
{
public:
    RigidBody(){;};

    float mass = 1.0f;
    float momentInertia = 0.4f * mass;
    Matrix3x3d inverseBodyInertia;
    Matrix3x3d inverseWorldInertia;
    Vector3D velocity = Vector3D(5.0f,0.0f,0.0f);
    Vector3D angularVelocity = Vector3D(0,0.2f,0.5f);
    Vector3D cogOffset = Vector3D(0,0,0);
    float dragCoefficient = 0.5f;
    float crossSectionArea = 0.1f;
    float mu = 0.5f;
    float elasticity = 0.6f;
    void setCollider(ColliderType type, float r = 0, float x = 0, float y = 0, float z = 0){collider_=  Collider(type,r,x,y,z);}
    Collider collider_{ColliderType::None};
    
private:
};

#endif /* RigidBody_hpp */
