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

class Engine
{
public:
    void RunApp();
    Engine(MTL::Device* device)
    :renderSystem_(device){}
    void RenderFrame(void* pass, void* drawable){renderSystem_.RenderFrame(pass, drawable);}
    void Update(int64_t dt);

private:
    World world_;
    RenderSystem renderSystem_;
    PhysicsSystem physicsSystem_;
    double accumulator_;
};



#endif /* Engine_hpp */
