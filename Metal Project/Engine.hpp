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
#include "InputSystem.hpp"
#include "Camera.h"
#include "Mesh.hpp"
#include <map>

constexpr float MOVEMENT_SPEED = 0.2f;
constexpr float MOUSE_SENSITIVIY = 0.002f;

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
    void KeyDown(int keyCode);
    void KeyUp(int keyCode);
    void MouseMoved(float dx, float dy);
    bool shouldQuit(){return shouldQuit_;}
    void requestQuit(){shouldQuit_ = true;}
    void loadMeshAsset(std::string filename, std::string assetName);
    
    std::pair<std::size_t,std::size_t> getMeshReference(std::string assetName);
    MeshAsset::physics getPhysics(std::string assetName);


private:
    std::vector<MeshAsset> meshAssets_;
    std::map<std::string,std::size_t> meshLookup_;
    World world_;
    RenderSystem renderSystem_;
    PhysicsSystem physicsSystem_;
    InputSystem inputSystem_;
    double accumulator_ = 0;
    bool shouldQuit_ = false;
};



#endif /* Engine_hpp */
