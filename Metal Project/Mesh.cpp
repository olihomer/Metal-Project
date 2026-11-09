//
//  Mesh.cpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#include "Mesh.hpp"
#include "metal-cpp/Foundation/Foundation.hpp"
#include "Matrix3x3.hpp"
#include <algorithm>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"


struct VertexKeyHash
{
    std::size_t operator()(const tinyobj::index_t& key) const noexcept
    {
        std::size_t seed = 0;
        
        auto combine = [&seed](int value)
        {
            seed ^= std::hash<int>{}(value)
            + 0x9e3779b9
            + (seed << 6)
            + (seed >> 2);
        };
        combine(key.vertex_index);
        combine(key.texcoord_index);
        combine(key.normal_index);
        
        return seed;
    }
};


MeshAsset::MeshAsset(std::string filename, std::string assetName)
: assetName_(assetName)
{
    NS::String* resourcePath = NS::Bundle::mainBundle()->resourcePath();
    if (resourcePath == nullptr)
    {
        throw std::runtime_error("Could not locate the application resource directory");
    }

    std::string objPath = resourcePath->utf8String();
    objPath += "/" + filename;
    Load(objPath.c_str());
    ComputeInertia();
}


void MeshAsset::Load(const char* filename)
{
    using namespace tinyobj;
    std::unordered_map<index_t, uint32_t, VertexKeyHash> lookup;
    
    std::cout << "Mesh::Load " << filename << std::endl;
    
    attrib_t attrib;
    std::vector<shape_t> shapes;
    std::vector<material_t> materials;
    std::string warnings;
    std::string errors;
    
    
    bool result = LoadObj(&attrib, &shapes, &materials, &warnings, &errors, filename);
    
    std::cout << "Result: " << result << std::endl;
    std::cout << "Warnings: " << warnings << std::endl;
    std::cout << "Errors: " << errors << std::endl;
    
    if (!result)
    {
        throw std::runtime_error("Could not load mesh: " + errors);
    }
    
    if (shapes.size() > 1)throw std::runtime_error("More than one shape");
    
    for (size_t s = 0; s < shapes.size(); s++)
    {
        size_t index_offset = 0;
        indices.reserve(shapes[s].mesh.indices.size());
        
        std::cout << "Faces: " << shapes[s].mesh.num_face_vertices.size() << std::endl;
        
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
        {
            unsigned char num_face_verts = shapes[s].mesh.num_face_vertices[f];
            
            for (unsigned char v = 0; v < num_face_verts; v++)
            {
                index_t index = shapes[s].mesh.indices[index_offset + v];
                
                auto it = lookup.find(index);
                
                if(lookup.find(index)!=lookup.end())
                {
                    indices.push_back(it->second);
                }
                else
                {
                    uint32_t newIndex = static_cast<uint32_t>(vertices.size());
                    
                    real_t vx = attrib.vertices[index.vertex_index * 3 + 0];
                    real_t vy = attrib.vertices[index.vertex_index * 3 + 1];
                    real_t vz = attrib.vertices[index.vertex_index * 3 + 2];
                    
                    real_t tx{},ty{},nx{},ny{},nz{};
                    if(index.texcoord_index * 2>-1)
                        tx = attrib.texcoords[index.texcoord_index * 2 + 0];
                    if(index.texcoord_index * 2 + 1>-1)
                        ty = attrib.texcoords[index.texcoord_index * 2 + 1];
                    
                    if(index.normal_index * 3 + 0>-1)
                        nx = attrib.normals[index.normal_index * 3 + 0];
                    if(index.normal_index * 3 + 1>-1)
                        ny = attrib.normals[index.normal_index * 3 + 1];
                    if(index.normal_index * 3 + 2>-1)
                        nz = attrib.normals[index.normal_index * 3 + 2];
                    
                    Vertex vertex(Vector3D(vx, vy, vz), Vector2D(tx, ty), Vector3D(nx,ny,nz));
                    vertices.push_back(vertex);
                    indices.push_back(newIndex);
                    lookup.emplace(index, newIndex);
                }
            }
            index_offset += num_face_verts;

        }
        
        std::cout << "Mesh from file created ok" << std::endl;
    }
}


void MeshAsset::ComputeInertia()
{
    Matrix3x3 canonical({1.0f/60.0f,1.0f/120.0f,1.0f/120.0f,1.0f/120.0f,1.0f/60.0f,1.0f/120.0f,1.0f/120.0f,1.0f/120.0f,1.0f/60.0f});
    Matrix3x3 cAccum({0,0,0,0,0,0,0,0,0});
    
    canonical.print();
    
    float xMean{}, yMean{}, zMean{};
    float xMin{}, yMin{}, zMin{};
    float xMax{}, yMax{}, zMax{};
    float volume{};
    float density = 1.0f;
    
    int faceCount = 0;
    for(std::size_t i = 0; i < indices.size(); i+=3)
    {
        Vector3D w0 = Vector3D(0,0,0);
        Vector3D w1 = vertices[indices[i]].vx_coords;
        Vector3D w2 = vertices[indices[i+1]].vx_coords;
        Vector3D w3 = vertices[indices[i+2]].vx_coords;

        xMean+=w1.x_+w2.x_+w3.x_;
        yMean+=w1.y_+w2.y_+w3.y_;
        zMean+=w1.z_+w2.z_+w3.z_;
        xMin=std::min({xMin,w1.x_,w2.x_,w3.x_});
        yMin=std::min({yMin,w1.y_,w2.y_,w3.y_});
        zMin=std::min({zMin,w1.z_,w2.z_,w3.z_});
        xMax=std::max({xMax,w1.x_,w2.x_,w3.x_});
        yMax=std::max({yMax,w1.y_,w2.y_,w3.y_});
        zMax=std::max({zMax,w1.z_,w2.z_,w3.z_});

        
        Vector3D a0 = w1 - w0;
        Vector3D a1 = w2 - w0;
        Vector3D a2 = w3 - w0;
    
        Matrix3x3 a({a0.x_,a1.x_,a2.x_,a0.y_,a1.y_,a2.y_,a0.z_,a1.z_,a2.z_});
        float detA = a.getDeterminant();
        volume += (detA/6.0f);
        Matrix3x3 aT = a.transpose();
        
        Matrix3x3 c = (a * (canonical * aT)) * detA;
        cAccum = cAccum + c;
        faceCount++;
    }
    xMean/=(faceCount*3);
    yMean/=(faceCount*3);
    zMean/=(faceCount*3);
    
    std::cout << "Mean: " << xMean << " " << yMean << " " << zMean << std::endl;
    std::cout << "Min: " << xMin << " " << yMin << " " << zMin << std::endl;
    std::cout << "Max: " << xMax << " " << yMax << " " << zMax << std::endl;
    std::cout << "Volume:" << volume << std::endl;

    cAccum.print();
    Matrix3x3 identity;
    Matrix3x3 I = ((identity * cAccum.getTrace()) - cAccum);
    I.print();
    
    Vector3D COG(xMean,yMean,zMean);
    Vector3D delta = COG * -1.0f;
    float mass = volume * density;
    
    //change of origin
    
    Matrix3x3 cPrime = cAccum + ((Matrix3x3(delta,COG) + Matrix3x3(COG,delta) + Matrix3x3(delta,delta)) * mass);
                                 
    cPrime.print();
    I = ((identity * cPrime.getTrace()) - cPrime);
    I.print();
    
}
