//
//  RigidBody.cpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#include "RigidBody.hpp"


void RigidBody::updateWorldInertia(Quaternion rotation)
{
    Matrix3x3d R = rotation.toMatrix3x3d();
    Matrix3x3d RT = R.transpose();
    inverseWorldInertia = RT * inverseBodyInertia * R;
    worldInertia = RT * bodyInertia * R;
}
