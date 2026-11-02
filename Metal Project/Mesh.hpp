//
//  Mesh.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef Mesh_hpp
#define Mesh_hpp

#include "Pool.hpp"
#include "Vector3D.h"
#include <vector>

class Mesh
{
public:
    Mesh(){;};
    void print(){std::cout << "Mesh  = " << std::endl;};

    std::vector<Vector3D> vertices;
private:
};

#endif /* Mesh_hpp */
