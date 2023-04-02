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
#ifndef GLDEBUG_H
#define GLDEBUG_H
///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <string>
#include <GL/glew.h>
#include <GL/glfw.h>
///////////////////////////////////////////////////////////////////////////////
std::string errorGL(GLenum error) {
  std::string msg;
  switch (error) {
#define Case(Token)  case Token: msg = #Token; break;
      Case(GL_INVALID_ENUM);
      Case(GL_INVALID_VALUE);
      Case(GL_INVALID_OPERATION);
      Case(GL_INVALID_FRAMEBUFFER_OPERATION);
      Case(GL_NO_ERROR);
      Case(GL_OUT_OF_MEMORY);
#undef Case
  }
  return msg;
}
void glCheckError(const std::string &file, int line) {
  GLenum error;
  while ((error = glGetError()) != GL_NO_ERROR) {
    std::cerr << "ERROR: file " << file << ", line " << line << ": " << errorGL(error) << std::endl;
  }
}
#ifdef OPENGL_DEBUG
#define GLCHECK glCheckError(__FILE__, __LINE__);
#else
#define GLCHECK
#endif
///////////////////////////////////////////////////////////////////////////////
#endif
///////////////////////////////////////////////////////////////////////////////
