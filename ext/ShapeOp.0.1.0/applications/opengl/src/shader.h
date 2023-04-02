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
#ifndef SHADER_H
#define SHADER_H
///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <vector>
#include <GL/glew.h>
#include <GL/glfw.h>
///////////////////////////////////////////////////////////////////////////////
/** \brief GLShader. Encapsulation of OpenGL Shaders.*/
class GLShader {
 public:
  GLShader(const std::string &vshader, const std::string &fshader) {
    init(vshader, fshader);
  }
  void bind() { glUseProgram(programID_); }
  void unbind() { glUseProgram(0); }
  GLuint getUniformLocation(const std::string &name) { return glGetUniformLocation(programID_, name.c_str()); }
  GLuint getProgramId() { return programID_; }
 private:
  void init(const std::string &vshader, const std::string &fshader) {
    /// Compile the Vertex Shader code from the file
    std::ifstream vertexShaderStream(vshader, std::ios::in);
    std::string  vertexShaderCode = std::string(std::istreambuf_iterator<char>(vertexShaderStream), std::istreambuf_iterator<char>());
    GLuint vertexShaderID = glCreateShader(GL_VERTEX_SHADER);
    char const *vShaderCode = vertexShaderCode.c_str();
    glShaderSource(vertexShaderID, 1, &vShaderCode, NULL);
    glCompileShader(vertexShaderID);
    getCompilationInfo(vertexShaderID);

    /// Compile the Fragment Shader code from the file
    std::ifstream fragmentShaderStream(fshader, std::ios::in);
    std::string fragmentShaderCode = std::string(std::istreambuf_iterator<char>(fragmentShaderStream), std::istreambuf_iterator<char>());
    GLuint fragmentShaderID = glCreateShader(GL_FRAGMENT_SHADER);
    char const *fShaderCode = fragmentShaderCode.c_str();
    glShaderSource(fragmentShaderID, 1, &fShaderCode, NULL);
    glCompileShader(fragmentShaderID);
    getCompilationInfo(fragmentShaderID);

    /// Link the program
    programID_ = glCreateProgram();
    glAttachShader(programID_, vertexShaderID);
    glAttachShader(programID_, fragmentShaderID);
    glLinkProgram(programID_);
    glDeleteShader(vertexShaderID);
    glDeleteShader(fragmentShaderID);
  }
  void getCompilationInfo(GLuint id) {
    GLint success = GL_FALSE;
    int logLength;
    glGetShaderiv(id, GL_COMPILE_STATUS, &success);
    glGetShaderiv(id, GL_INFO_LOG_LENGTH, &logLength);
    if (!success) {
      std::vector<char> error(logLength);
      glGetShaderInfoLog(id, logLength, NULL, error.data());
      std::string errorStr(error.begin(), error.end());
      std::cout << errorStr << std::endl;
    }
  }
  GLuint programID_;
};
///////////////////////////////////////////////////////////////////////////////
#endif
///////////////////////////////////////////////////////////////////////////////
