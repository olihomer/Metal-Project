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

class RigidBody
{
public:
    RigidBody(){;};

    void print(){std::cout << "RigidBody  = " << std::endl;};

    float mass = 1.0;
    float momentInertia = 0.0;
    Vector3D velocity = Vector3D(0,0,0);
    Vector3D cogOffset = Vector3D(0,0,0);
    float dragCoefficient = 0.5;
    float crossSectionArea = 1.0;
    
    
private:
};

#endif /* RigidBody_hpp */
