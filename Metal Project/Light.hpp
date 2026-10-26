//
//  Light.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef Light_hpp
#define Light_hpp

#include "Pool.hpp"

class Light
{
public:
    Light(ECS::Handle<Light> handle){handle_ = handle;};
private:
    ECS::Handle<Light> handle_;
};

#endif /* Light_hpp */
