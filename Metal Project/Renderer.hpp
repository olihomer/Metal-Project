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

#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>


class Renderer
{
public:
    Renderer(MTL::Device* device);
    void RunApp();
    void RenderFrame(void* pass, void* drawable);
    
private:
    //Metal objects
    NS::SharedPtr<MTL::Device> device_;
    NS::SharedPtr<MTL::CommandQueue> commandQueue_;
    NS::SharedPtr<MTL::Library> defaultLibrary_;
    NS::SharedPtr<MTL::RenderPipelineState> pipelineState_;
    
};




#endif // !Renderer_hpp
