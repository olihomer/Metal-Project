//
//  Renderer.hpp
//  Metal Project
//
//  Created by Oliver Homer on 12/10/2026.
//

#ifndef Renderer_hpp
#define Renderer_hpp

#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include "Pool.hpp"
#include "Transform.hpp"
#include "World.hpp"
#include "Matrix4x4.h"

#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>

constexpr NS::UInteger VERTEXBUFFER = 0;
constexpr NS::UInteger VIEWPROJBUFFER = 1;
constexpr NS::UInteger MODELBUFFER = 2;

struct alignas(16) ViewProjection
{
    Matrix4x4 viewMatrix;
    Matrix4x4 projectionMatrix;
};

class RenderSystem
{
public:
    RenderSystem(MTL::Device* device);
    void RenderFrame(void* pass, void* drawable, World& world);
    
private:
    //Metal objects
    NS::SharedPtr<MTL::Device> device_;
    NS::SharedPtr<MTL::CommandQueue> commandQueue_;
    NS::SharedPtr<MTL::Library> defaultLibrary_;
    NS::SharedPtr<MTL::RenderPipelineState> pipelineState_;
    
};




#endif // !Renderer_hpp
