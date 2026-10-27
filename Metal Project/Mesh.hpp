//
//  Mesh.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef Mesh_hpp
#define Mesh_hpp

#include "Pool.hpp"

class Mesh
{
public:
    Mesh(ECS::Handle<Mesh> handle){handle_ = handle;};
    void print(){std::cout << "Mesh handle = " << handle_.slotIndex << std::endl;};

private:
    ECS::Handle<Mesh> handle_;
};

#endif /* Mesh_hpp */
