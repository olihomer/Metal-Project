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
private:
    ECS::Handle<Transform> handle_;
    Vector3D position_;
    Quaternion rotation_;
    
};
#endif /* Transform_hpp */
