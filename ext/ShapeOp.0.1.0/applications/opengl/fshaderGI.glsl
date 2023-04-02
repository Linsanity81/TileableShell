///////////////////////////////////////////////////////////////////////////////
// This file is part of ShapeOp, a lightweight C++ library
// for static and dynamic geometry processing.
//
// Copyright (C) 2014 Sofien Bouaziz <sofien.bouaziz@gmail.com>
// Copyright (C) 2014 LGG EPFL
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
///////////////////////////////////////////////////////////////////////////////
#version 330 core
layout (location = 0) out vec3 color;
in vec2 uv;
uniform mat4 inverseProjection;
uniform sampler2D normalDepthTex;
uniform sampler2D directTex;
uniform sampler2D randomTex;
vec3 positionFromDepth(in vec2 uv,
                       in float depth) {
     vec3 ray = mat3(inverseProjection)*vec3((uv-0.5)*2.0, 1.0);
     return vec3(ray.xy*depth, -depth);
}
//Ambient Occlusion and GI form factor
vec4 formFactor(in vec3 position1,
                in vec3 normal1,
                in vec3 position2,
                in vec3 normal2,
                in vec3 diffuse2) {
      vec3 vv = position2 - position1;
      float rdSq = dot(vv,vv) + 1e-16;
      vv *= inversesqrt(rdSq);
//      rdSq *= 2.0;
      float c1 = max(dot(normal1, vv), 0.0);
      float c2 = max(dot(normal2, -vv), 0.0);
      float ao = (1.0 - c2) * c1 * (1.0 - inversesqrt(1.0/rdSq + 1.0));
      float gi = c2 * c1/(rdSq + 1.0);
      return vec4(ao, gi*diffuse2);
}
vec4 formFactor(in vec3 position1,
                in vec3 normal1,
                in vec2 uv2) {
    vec3 diffuse2 = texture(directTex, uv2).xyz;
    vec4 normalDepth2 = texture(normalDepthTex, uv2);
    vec3 normal2 = normalDepth2.xyz;
    vec3 position2 = positionFromDepth(uv2, normalDepth2.w);
    return formFactor(position1, normal1, position2, normal2, diffuse2);
}
void main() {
    vec3 diffuse1 = texture(directTex, uv).xyz;
    vec4 normalDepth1 = texture(normalDepthTex, uv);
    vec3 normal1 = normalDepth1.xyz;
    vec3 position1 = positionFromDepth(uv, normalDepth1.w);
    float s = 10.0;
    vec2 inc = s*vec2(1.0/1024.0, 1.0/768.0);
    float mu = 5.0/128.0;
    vec2 fres = mu*vec2(1024.0, 768.0);
    vec2 random = texture(randomTex, uv*fres).xy;
    random = inc*(random*2.0-vec2(1.0))/2.0;
    vec4 g = vec4(0.0);
    vec2 p = inc;
    float cd = normalDepth1.w;
    for(float i=0.0; i<8.0; ++i) {
        vec2 np =  (p+random.xy)/cd;
        g+= formFactor(position1, normal1, uv + vec2(np.x, np.y));
        g+= formFactor(position1, normal1, uv + vec2(np.x, -np.y));
        g+= formFactor(position1, normal1, uv + vec2(-np.x, np.y));
        g+= formFactor(position1, normal1, uv + vec2(-np.x, -np.y));
        g+= formFactor(position1, normal1, uv + vec2(0.0, np.y));
        g+= formFactor(position1, normal1, uv + vec2(0.0, -np.y));
        g+= formFactor(position1, normal1, uv + vec2(np.x, 0.0));
        g+= formFactor(position1, normal1, uv + vec2(-np.x, 0.0));
        p += inc;
    }
    g /= 64.0;
//    diffuse1 = vec3(1.0);
    color = diffuse1-vec3(1.2*g.x)+5.0*g.yzw;
}

