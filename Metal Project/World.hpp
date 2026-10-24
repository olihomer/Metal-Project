//
//  World.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef World_hpp
#define World_hpp

#include <vector>
#include "Pool.hpp"
#include "RigidBody.hpp"
#include "Mesh.hpp"
#include "Light.hpp"
#include "Entity.hpp"

class World
{
public:
    World();
    Entity newEntity();
private:
    Pool<Entity> entities_;
    Pool<RigidBody> rigidBodyPool_;
    Pool<Mesh> meshPool_;
    Pool<Light> lightPool_;
};



#endif /* World_hpp */
