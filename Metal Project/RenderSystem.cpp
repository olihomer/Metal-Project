//
//  RenderSystem.cpp
//  Metal Project
//
//  Created by Oliver Homer on 12/10/2026.
//

#define NS_PRIVATE_IMPLEMENTATION
#define MTL_PRIVATE_IMPLEMENTATION
#define CA_PRIVATE_IMPLEMENTATION


#include "RenderSystem.hpp"
#include <fstream>
#include <string>
#include <sstream>
#include <Foundation/Foundation.hpp>
#include <Metal/Metal.hpp>
#include <QuartzCore/QuartzCore.hpp>
#include "World.hpp"


RenderSystem::RenderSystem(MTL::Device* device)
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


void RenderSystem::RenderFrame(void* passPtr, void* drawablePtr, World& world)
{
    auto* passDescriptor = reinterpret_cast<MTL::RenderPassDescriptor*>(passPtr);
    auto* drawable = reinterpret_cast<MTL::Drawable*>(drawablePtr);
    
    auto* commandBuffer = commandQueue_->commandBuffer();
    auto* commandEncoder = commandBuffer->renderCommandEncoder(passDescriptor);

    
    //do world stuff
    
    float *vertices = nullptr;
    std::vector<Vertex>verticesVector;
    
    world.each<Transform, Mesh>([&](ECS::Handle<ECS::Entity> entity, Transform& transform, Mesh& mesh)
    {
        verticesVector.insert(verticesVector.end(),mesh.vertices.begin(),mesh.vertices.end());
    });
        
    if(verticesVector.size()>0)vertices = new float[verticesVector.size() * 4];
          
    std::size_t i = 0;
    
    world.each<Transform, Mesh>([&](ECS::Handle<ECS::Entity> entity, Transform& transform, Mesh& mesh)
    {
        float x = transform.position_.x_/20.0f;
        float y = transform.position_.y_/20.0f;
        Vector3D worldTransform(x,y,0);
                
        for(Vertex vertex: mesh.vertices)
        {
            vertex.vx_coords = vertex.vx_coords + worldTransform;
            vertices[i++] = vertex.vx_coords.x_;
            vertices[i++] = vertex.vx_coords.y_;
            vertices[i++] = vertex.vx_coords.z_;
            vertices[i++] = 1.0f;
        }
    });
    
    NS::UInteger length = verticesVector.size() * 4;
    MTL::Buffer* buffer = nullptr;
    
    commandEncoder->setRenderPipelineState(pipelineState_.get());
    
    if(length > 0)
    {
        buffer = device_->newBuffer(vertices, length * sizeof(float), MTL::ResourceStorageModeShared);
        commandEncoder->setVertexBuffer(buffer, 0, 0);
        commandEncoder->drawPrimitives(MTL::PrimitiveTypeTriangle, NS::UInteger(0), NS::UInteger(length/3));
    }

    commandEncoder->endEncoding();
    commandBuffer->presentDrawable(drawable);
    commandBuffer->commit();
    if(buffer)buffer->release();
    if(vertices)delete[] vertices;
}
