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
    commandEncoder->setRenderPipelineState(pipelineState_.get());

    //get camera transforms and send to vertex shader buffer
    
    auto* camera = world.getCamera();
    
    ViewProjection viewProjection{camera->getCameraTransform(), camera->getPerspectiveTransform()};
    
    commandEncoder->setVertexBytes(&viewProjection, sizeof(viewProjection), VIEWPROJBUFFER);
    
    // iterate through meshes
    
    float *vertices = nullptr;
              
    world.each<Transform, Mesh>([&](ECS::Handle<ECS::Entity> entity, Transform& transform, Mesh& mesh)
    {
        if(mesh.vertices.size()>0)
        {
            vertices = new float[mesh.vertices.size() * 4];
            std::size_t i = 0;
            
            Matrix4x4 modelMatrix = transform.rotation_.toMatrix();
            modelMatrix.setTranslation(transform.position_);
            
            commandEncoder->setVertexBytes(&modelMatrix, sizeof(modelMatrix), MODELBUFFER);
            
            for(Vertex vertex: mesh.vertices)
            {
                vertices[i++] = vertex.vx_coords.x_;
                vertices[i++] = vertex.vx_coords.y_;
                vertices[i++] = vertex.vx_coords.z_;
                vertices[i++] = 1.0f;
            }
            
            NS::UInteger length = mesh.vertices.size() * 4;
            MTL::Buffer* buffer = nullptr;

            if(length > 0)
            {
                buffer = device_->newBuffer(vertices, length * sizeof(float), MTL::ResourceStorageModeShared);
                commandEncoder->setVertexBuffer(buffer, 0, VERTEXBUFFER);
                commandEncoder->drawPrimitives(MTL::PrimitiveTypeTriangle, NS::UInteger(0), mesh.vertices.size());
                if(buffer)buffer->release();
                if(vertices)delete[] vertices;
            }
        }
    });
    
    commandEncoder->endEncoding();
    commandBuffer->presentDrawable(drawable);
    commandBuffer->commit();
}
