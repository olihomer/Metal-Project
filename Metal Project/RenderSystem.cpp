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
#include "Mesh.hpp"


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
    pipelineDesc->setDepthAttachmentPixelFormat(MTL::PixelFormatDepth32Float);

    NS::Error* error;
    pipelineState_ = NS::TransferPtr(device_->newRenderPipelineState(pipelineDesc,&error));
    if(error)
    {
        std::cout << error->code() << std::endl;
        throw std::runtime_error("couldn't create pipeline");
    }
    
    auto* depthDesc = MTL::DepthStencilDescriptor::alloc()->init();

    depthDesc->setDepthCompareFunction(
        MTL::CompareFunctionLess
    );

    depthDesc->setDepthWriteEnabled(true);

    depthState_ = NS::TransferPtr(
        device_->newDepthStencilState(depthDesc)
    );

    depthDesc->release();
    
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
    commandEncoder->setDepthStencilState(depthState_.get());

    //get camera transforms and send to vertex shader buffer
    
    auto* camera = world.getCamera();
    
    ViewProjection viewProjection{camera->getCameraTransform(), camera->getPerspectiveTransform()};
    
    commandEncoder->setVertexBytes(&viewProjection, sizeof(viewProjection), VIEWPROJBUFFER);
    
    // iterate through meshes
                  
    world.each<Transform, Mesh>([&](ECS::Handle<ECS::Entity> entity, Transform& transform, Mesh& mesh)
    {
        Matrix4x4 modelMatrix = transform.rotation_.toMatrix();
        modelMatrix.setTranslation(transform.position_);
        
        commandEncoder->setVertexBytes(&modelMatrix, sizeof(modelMatrix), MODELBUFFER);
        
        auto vertexBuffer = gpuMeshes_[mesh.gpuMeshIndex_].vertexBuffer_.get();
        auto indexBuffer = gpuMeshes_[mesh.gpuMeshIndex_].indexBuffer_.get();
        auto size = gpuMeshes_[mesh.gpuMeshIndex_].size_;
        commandEncoder->setVertexBuffer(vertexBuffer, 0, VERTEXBUFFER);
        commandEncoder->drawIndexedPrimitives(MTL::PrimitiveTypeTriangle, size, MTL::IndexTypeUInt32, indexBuffer, 0);
    }
        
    );
    
    commandEncoder->endEncoding();
    commandBuffer->presentDrawable(drawable);
    commandBuffer->commit();
}


void RenderSystem::prepareMeshBuffers(MeshAsset& meshAsset)
{
    if(meshAsset.vertices.size()==0)
    {
        throw std::runtime_error("Mesh has no vertices");
    }
    
    std::vector<float> vertices;
    vertices.resize(meshAsset.vertices.size() * 4);
    std::size_t i = 0;
        
    for(Vertex vertex: meshAsset.vertices)
    {
        vertices[i++] = vertex.vx_coords.x_;
        vertices[i++] = vertex.vx_coords.y_;
        vertices[i++] = vertex.vx_coords.z_;
        vertices[i++] = 1.0f;
    }
    
    NS::UInteger length = meshAsset.vertices.size() * 4;
    gpuMesh temp;
    
    if(length > 0)
    {
        temp.size_ = meshAsset.indices.size();
        temp.vertexBuffer_= NS::TransferPtr(device_->newBuffer(vertices.data(), length * sizeof(float), MTL::ResourceStorageModeShared));
        temp.indexBuffer_ = NS::TransferPtr(device_->newBuffer(meshAsset.indices.data(), meshAsset.indices.size() * sizeof(unsigned int), MTL::ResourceStorageModeShared));
        gpuMeshes_.push_back(temp);
        meshAsset.gpuMeshIndex_ = gpuMeshes_.size()-1;
    }
}
