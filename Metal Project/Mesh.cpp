//
//  Mesh.cpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#include "Mesh.hpp"
#include "metal-cpp/Foundation/Foundation.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"


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
}


void MeshAsset::Load(const char* filename)
{
    using namespace tinyobj;
    
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
        //m_list_vertices.reserve(shapes[s].mesh.indices.size());
        indices.reserve(shapes[s].mesh.indices.size());
        
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++)
        {
            unsigned char num_face_verts = shapes[s].mesh.num_face_vertices[f];

            for (unsigned char v = 0; v < num_face_verts; v++)
            {
                index_t index = shapes[s].mesh.indices[index_offset + v];

                real_t vx = attrib.vertices[index.vertex_index * 3 + 0];
                real_t vy = attrib.vertices[index.vertex_index * 3 + 1];
                real_t vz = attrib.vertices[index.vertex_index * 3 + 2];

                real_t tx = attrib.texcoords[index.texcoord_index * 2 + 0];
                real_t ty = attrib.texcoords[index.texcoord_index * 2 + 1];

                real_t nx = attrib.normals[index.normal_index * 3 + 0];
                real_t ny = attrib.normals[index.normal_index * 3 + 1];
                real_t nz = attrib.normals[index.normal_index * 3 + 2];

                Vertex vertex(Vector3D(vx, vy, vz), Vector2D(tx, ty), Vector3D(nx,ny,nz));
                vertices.push_back(vertex);
                indices.push_back((unsigned int)index_offset+v);
            }
            index_offset += num_face_verts;
        }
    }

    std::cout << "Mesh from file created ok" << std::endl;
}
