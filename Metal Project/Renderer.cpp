//
//  AppEngine.cpp
//  Metal Project
//
//  Created by Oliver Homer on 12/10/2026.
//

#define NS_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION
#define CA_PRIVATE_IMPLEMENTATION


#include "Renderer.hpp"
#include <fstream>
#include <string>
#include <sstream>
#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>
#include "World.hpp"



Renderer::Renderer(MTL::Device* device)
:device_(NS::RetainPtr(device))
{
    //Initialise Metal Objects
    commandQueue_ = NS::TransferPtr(device_->newCommandQueue());
    defaultLibrary_ = NS::TransferPtr(device_->newDefaultLibrary());
    
    auto* functionName = NS::String::string("vertexMain",NS::UTF8StringEncoding);
    auto* vertexFunction = defaultLibrary_->newFunction(functionName);

    functionName = NS::String::string("fragmentMain",NS::UTF8StringEncoding);
    auto* fragmentFunction = defaultLibrary_->newFunction(functionName);

    auto* pipelineDesc = MTL::RenderPipelineDescriptor::alloc()->init();
    pipelineDesc->setFragmentFunction(fragmentFunction);
    pipelineDesc->setVertexFunction(vertexFunction);
    pipelineDesc->colorAttachments()->object(0)->setPixelFormat(MTL::PixelFormatBGRA8Unorm);

    NS::Error* error;
    pipelineState_ = NS::TransferPtr(device_->newRenderPipelineState(pipelineDesc,&error));
    if(error)
    {
        std::cout << error->code() << std::endl;
        throw std::runtime_error("couldn't create pipeline");
    }
    
    pipelineDesc->release();
    fragmentFunction->release();
    vertexFunction->release();

}

void Renderer::RunApp()
{
    std::cout << "RunApp()" << std::endl;
    World myWorld;
    
    auto myEntity = myWorld.newEntity("Dog");

    myWorld.addComponent<Light>(myEntity);
    myWorld.addComponent<Transform>(myEntity,Vector3D(1,2,3));
    myWorld.addComponent<RigidBody>(myEntity);
    myWorld.addComponent<Mesh>(myEntity);

    myWorld.printEntities();
    myWorld.printComponents<Light>();
    myWorld.printComponents<Transform>();
    myWorld.printComponents<RigidBody>();
    myWorld.printComponents<Mesh>();

    myWorld.deleteEntity(myEntity);

    myWorld.printComponents<Light>();
    myWorld.printComponents<Transform>();
    myWorld.printComponents<RigidBody>();
    myWorld.printComponents<Mesh>();
    
}

void Renderer::RenderFrame(void* passPtr, void* drawablePtr)
{
    auto* passDescriptor = reinterpret_cast<MTL::RenderPassDescriptor*>(passPtr);
    auto* drawable = reinterpret_cast<MTL::Drawable*>(drawablePtr);
    
    auto* commandBuffer = commandQueue_->commandBuffer();
    auto* commandEncoder = commandBuffer->renderCommandEncoder(passDescriptor);
    
    commandEncoder->setRenderPipelineState(pipelineState_.get());
    commandEncoder->drawPrimitives(MTL::PrimitiveTypeTriangle, NS::UInteger(0), NS::UInteger(3));
    
    commandEncoder->endEncoding();
    commandBuffer->presentDrawable(drawable);
    commandBuffer->commit();
}
