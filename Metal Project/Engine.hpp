//
//  Engine.hpp
//  Metal Project
//
//  Created by Oliver Homer on 29/10/2026.
//

#ifndef Engine_hpp
#define Engine_hpp

#include "World.hpp"
#include "RenderSystem.hpp"
#include "PhysicsSystem.hpp"
#include "Camera.h"

class Engine
{
public:
    void RunApp();
    Engine(MTL::Device* device)
    :renderSystem_(device)
    {
        std::cout << "Constructor: " << this << std::endl;
        RunApp();
        world_.printEntities();
    }
    void RenderFrame(void* pass, void* drawable){renderSystem_.RenderFrame(pass, drawable, world_);}
    void Update(double dt);

private:
    World world_;
    RenderSystem renderSystem_;
    PhysicsSystem physicsSystem_;
    double accumulator_ = 0;
};



#endif /* Engine_hpp */
