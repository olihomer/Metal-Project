//
//  triangle.metal
//  Metal Project
//
//  Created by Oliver Homer on 17/10/2026.
//

#include <metal_stdlib>
using namespace metal;

struct CameraMatrices
{
    float4x4 view;
    float4x4 projection;
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


VertexData vertex vertexMain(constant float4* vertexbuffer [[buffer(0)]], uint vertexID [[vertex_id]], constant CameraMatrices& camera [[buffer(1)]], constant float4x4& model [[buffer(2)]])
{
    VertexData data;
    data.position = model * vertexbuffer[vertexID];
    data.position = camera.view * data.position;
    data.position = camera.projection * data.position;
    
    data.color = colors[vertexID % 3];
    return data;
}


half4 fragment fragmentMain(VertexData frag [[stage_in]])
{
    return half4(frag.color, 1.0);
}


