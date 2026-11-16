//
//  grid.metal
//  Metal Project
//
//  Created by Oliver Homer on 16/11/2026.
//

#include <metal_stdlib>

using namespace metal;

struct CameraMatrices
{
    float4x4 view;
    float4x4 projection;
};

vertex float4 gridVertex(
    uint vid [[vertex_id]],
    constant CameraMatrices& camera [[buffer(1)]])
{
    const int N = 100;
    const float spacing = 1.0;

    // Two vertices per line
    uint line = vid / 2;
    uint endpoint = vid % 2;

    const uint linesPerDirection = 2 * N + 1;

    float x, z;

    if (line < linesPerDirection)
    {
        // Lines parallel to X
        x = endpoint == 0 ? -N : N;
        z = float(line) - N;
    }
    else
    {
        // Lines parallel to Z
        x = float(line - linesPerDirection) - N;
        z = endpoint == 0 ? -N : N;
    }

    float4 worldPos = float4(
        x * spacing, 0.001, z * spacing, 1.0
    );

    worldPos = camera.view * worldPos;
    worldPos = camera.projection * worldPos;
    return  worldPos;
}

fragment float4 gridFragment()
{
    return float4(0.4, 0.4, 0.4, 1.0);
}


