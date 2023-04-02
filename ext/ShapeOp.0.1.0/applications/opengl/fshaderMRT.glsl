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
layout (location = 0) out vec4 normalDepth_c;
layout (location = 1) out vec3 diffuse_c;
in float depth_mv;
in vec3 normal_mv;
uniform vec3 diffuse;
void main() {
    normalDepth_c = vec4(normalize(normal_mv), depth_mv);
    diffuse_c = diffuse;
}
