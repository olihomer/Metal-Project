//
//  triangle.metal
//  Metal Project
//
//  Created by Oliver Homer on 17/10/2026.
//

#include <metal_stdlib>
using namespace metal;

constant float4 positions[] =
{
    float4(-0.75, -0.75, 0.0, 1.0),
    float4(0.75, -0.75, 0.0, 1.0),
    float4(0.0, 0.75, 0.0, 1.0)
};

constant half3 colors[] =
{
    half3(1.0,0.0,0.0),
    half3(0.0,1.0,0.0),
    half3(0.0,0.0,1.0)
};

struct VertexData
{
    float4 position [[position]];
    half3 color;
};

VertexData vertex vertexMain(uint vertexID [[vertex_id]])
{
    VertexData data;
    data.position = positions[vertexID];
    data.color = colors[vertexID];
    return data;
}

half4 fragment fragmentMain(VertexData frag [[stage_in]])
{
    return half4(frag.color, 1.0);
}
