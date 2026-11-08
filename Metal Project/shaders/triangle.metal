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

struct Vertex
{
    float4 position;
    float3 normal;
};

struct VertexData
{
    float4 position [[position]];
    float3 normal;
    half3 color;
};


VertexData vertex vertexMain(constant Vertex* vertices [[buffer(0)]], uint vertexID [[vertex_id]], constant CameraMatrices& camera [[buffer(1)]], constant float4x4& model [[buffer(2)]])
{
    VertexData data;
    Vertex v = vertices[vertexID];
    data.position = model * v.position;
    data.position = camera.view * data.position;
    data.position = camera.projection * data.position;
    data.normal = v.normal;
    data.color = colors[vertexID % 3];
    return data;
}


half4 fragment fragmentMain(VertexData frag [[stage_in]], uint primitive [[primitive_id]])
{
    //return half4(colors[primitive%3], 1.0);
    
    float kd = 0.7; //diffuse reflection
    float3 id = float3(colors[primitive%3]); //diffuse light source
    float3 light_direction = float3(2.0,5.0,-5.0);
    
    float amount_diffuse_light = max(0.0,dot(light_direction, frag.normal));
    
    float3 diffuse_light = kd * id * amount_diffuse_light; //diffuse component
    
    return half4(half3(diffuse_light),1.0);
    
    
}


