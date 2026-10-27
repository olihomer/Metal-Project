//
//  RigidBody.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef RigidBody_hpp
#define RigidBody_hpp

#include "Pool.hpp"

class RigidBody
{
public:
    RigidBody(ECS::Handle<RigidBody> handle){handle_ = handle;};
    void print(){std::cout << "RigidBody handle = " << handle_.slotIndex << std::endl;};

private:
    ECS::Handle<RigidBody> handle_;
};

#endif /* RigidBody_hpp */
