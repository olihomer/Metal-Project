//
//  Mesh.hpp
//  Metal Project
//
//  Created by Oliver Homer on 20/10/2026.
//

#ifndef Mesh_hpp
#define Mesh_hpp

#include "Pool.hpp"
#include "Vector2D.h"
#include "Vector3D.h"
#include "Vector3Dd.hpp"
#include "Vector4D.h"
#include "Matrix3x3d.hpp"
#include <vector>
#include <string>


struct Vertex
{
    Vertex() {};
    Vertex(float x, float y, float z)
    {
        vx_coords = Vector3D{x,y,z};
    }
    Vertex(Vector3D vx, Vector2D tx, Vector3D nx,Vector4D col_rgba) : vx_coords(vx), tx_coords(tx),nx_coords(nx),col_rgba(col_rgba) {};
    Vertex(Vector3D vx, Vector2D tx, Vector3D nx) : vx_coords(vx), tx_coords(tx), nx_coords(nx), col_rgba() {};

    Vector3D vx_coords = {};
    Vector2D tx_coords = {};
    Vector3D nx_coords = {};
    Vector4D col_rgba = {};
};



class MeshAsset
{
public:
    MeshAsset(std::string filename, std::string assetName, double scaleFactor);
    void Load(const char* filename, double scaleFactor);

    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::size_t gpuMeshIndex_;
    struct physics
    {
        Matrix3x3d inverseBodyInertia_;
        double massAtUnitDensity_;
    }physics_;

private:
    std::string assetName_;
    void ComputeInertia();
};


class Mesh
{
public:
    Mesh(std::pair<std::size_t,std::size_t> meshRef)
    {
        meshIndex_ = meshRef.first;
        gpuMeshIndex_ = meshRef.second;
    }
    std::size_t meshIndex_;
    std::size_t gpuMeshIndex_;
    
private:
};

#endif /* Mesh_hpp */

