//
//  Vector3Dd.cpp
//  Metal Project
//
//  Created by Oliver Homer on 10/11/2026.
//

#include "Vector3Dd.hpp"
#include "Vector3D.h"
#include "Matrix3x3d.hpp"

Vector3Dd::Vector3Dd(const Vector3D& v)
: x_(v.x_), y_(v.y_), z_(v.z_)
{}


Vector3Dd Vector3Dd::operator*(const Matrix3x3d& rhs)
{
    Vector3Dd out;
    out.x_ = rhs.mat_[0][0] * x_ + rhs.mat_[1][0] * y_ + rhs.mat_[2][0] * z_;
    out.y_ = rhs.mat_[0][1] * x_ + rhs.mat_[1][1] * y_ + rhs.mat_[2][1] * z_;
    out.z_ = rhs.mat_[0][2] * x_ + rhs.mat_[1][2] * y_ + rhs.mat_[2][2] * z_;
    return out;
}
