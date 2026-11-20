//
//  Transform.hpp
//  Metal Project
//
//  Created by Oliver Homer on 24/10/2026.
//

#ifndef Transform_hpp
#define Transform_hpp

#include "Pool.hpp"
#include "Vector3D.h"
#include "Quaternion.h"
#include "Matrix4x4.h"


class Transform
{
    
public:
    Transform(Vector3D position = {}, Quaternion rotation = {})
    :position_(position), rotation_(rotation)
    {};
    void print(){std::cout << "Transform position = " << position_.x_ << "," << position_.y_ << "," << position_.z_ << std::endl;}
public:
    Vector3D position_;
    Quaternion rotation_;
    Matrix4x4 getWorldTransform()
    {
        auto matrix = rotation_.toMatrix();
        matrix.setTranslation(position_);
        return matrix;
    }
    
};
#endif /* Transform_hpp */
