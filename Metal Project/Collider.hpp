//
//  Collider.hpp
//  Metal Project
//
//  Created by Oliver Homer on 02/11/2026.
//

#ifndef Collider_hpp
#define Collider_hpp

enum class ColliderType{None, Sphere, Cube, Cuboid, Capsule};

class Collider
{
public:
    ColliderType type_;
    float r_; // used for sphere, cube, capsule
    float x_; // used for cuboid
    float y_; // used for capsule and cuboid
    float z_; // used for cuboid
    Collider(ColliderType type, float r = 0, float x = 0, float y = 0, float z = 0)
    :type_(type), r_(r), x_(x), y_(y), z_(z)
    {
        
    }
};



#endif /* Collider_hpp */
