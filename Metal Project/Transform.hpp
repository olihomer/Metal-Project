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


class Transform
{
    
public:
    Transform(Vector3D position = {}, Quaternion rotation = {})
    :position_(position), rotation_(rotation)
    {};
    void print(){std::cout << "Transform position = " << position_.x_ << "," << position_.y_ << "," << position_.z_ << std::endl;}
private:
    Vector3D position_;
    Quaternion rotation_;
    
};
#endif /* Transform_hpp */
