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
uniform sampler2D giTex;
void main() {
  // The parameters are hardcoded for now, but could be
  // made into uniforms to control fromt the program.
  float FXAA_SPAN_MAX = 8.0;
  float FXAA_REDUCE_MUL = 1.0/8.0;
  float FXAA_REDUCE_MIN = (1.0/128.0);
  vec2 texcoordOffset = vec2(1.0/1024.0, 1.0/768.0); //1.0/textureSize(giTex, 0);
  vec3 rgbNW = textureOffset(giTex, uv, ivec2(-1, -1)).xyz;
  vec3 rgbNE = textureOffset(giTex, uv, ivec2(+1, -1)).xyz;
  vec3 rgbSW = textureOffset(giTex, uv, ivec2(-1, +1)).xyz;
  vec3 rgbSE = textureOffset(giTex, uv, ivec2(+1, +1)).xyz;
  vec3 rgbM  = texture(giTex, uv).xyz;

  vec3 luma = vec3(0.299, 0.587, 0.114);
  float lumaNW = dot(rgbNW, luma);
  float lumaNE = dot(rgbNE, luma);
  float lumaSW = dot(rgbSW, luma);
  float lumaSE = dot(rgbSE, luma);
  float lumaM  = dot( rgbM, luma);

  float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
  float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));

  vec2 dir;
  dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));
  dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));

  float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * (0.25 * FXAA_REDUCE_MUL), FXAA_REDUCE_MIN);

  float rcpDirMin = 1.0/(min(abs(dir.x), abs(dir.y)) + dirReduce);

  dir = min(vec2(FXAA_SPAN_MAX,  FXAA_SPAN_MAX),
        max(vec2(-FXAA_SPAN_MAX, -FXAA_SPAN_MAX), dir * rcpDirMin)) * texcoordOffset;

  vec3 rgbA = (1.0/2.0) * (
              texture(giTex, uv + dir * (1.0/3.0 - 0.5)).xyz +
              texture(giTex, uv + dir * (2.0/3.0 - 0.5)).xyz);
  vec3 rgbB = rgbA * (1.0/2.0) + (1.0/4.0) * (
              texture(giTex, uv + dir * (0.0/3.0 - 0.5)).xyz +
              texture(giTex, uv + dir * (3.0/3.0 - 0.5)).xyz);
  float lumaB = dot(rgbB, luma);

  if((lumaB < lumaMin) || (lumaB > lumaMax)){
    color = rgbA;
  } else {
    color = rgbB;
  }
}

