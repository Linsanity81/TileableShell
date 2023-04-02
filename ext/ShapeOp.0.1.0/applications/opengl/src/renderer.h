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
#ifndef RENDERER_H
#define RENDERER_H
///////////////////////////////////////////////////////////////////////////////
#include <memory>
#include "shader.h"
#include "mesh.h"
#include "camera.h"
///////////////////////////////////////////////////////////////////////////////
/** \brief GLRenderer. A OpenGL renderer supporting various shaders. */
class GLRenderer {
 public:
  GLRenderer(int width, int height) :
    shaderMRT_("vshaderMRT.glsl", "fshaderMRT.glsl"),
    shaderShadow_("vshaderShadow.glsl", "fshaderShadow.glsl"),
    shaderDirect_("vshaderDirect.glsl", "fshaderDirect.glsl"),
    shaderGI_("vshaderGI.glsl", "fshaderGI.glsl"),
    shaderFXAA_("vshaderFXAA.glsl", "fshaderFXAA.glsl"),
    texWidth_(width),
    texHeight_(height) {
    init();
  }
  void addMesh(const std::shared_ptr<Object> &m) { meshes_.push_back(m); }
  void setLight(const std::shared_ptr<Camera> &l) { light_ = l; }
  void setCamera(const std::shared_ptr<Camera> &c) { camera_ = c; }
  void display() {
    //Multi render targets rendering
    GLenum drawBuffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
    glEnable(GL_DEPTH_TEST);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboMRT_);
    glDrawBuffers(2, drawBuffers);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    shaderMRT_.bind();
    glUniformMatrix4fv(shaderMRT_.getUniformLocation("view"), 1, GL_FALSE, camera_->getView().data());
    glUniformMatrix4fv(shaderMRT_.getUniformLocation("projection"), 1, GL_FALSE, camera_->getProjection().data());
    for (int i = 0; i < static_cast<int>(meshes_.size()); ++i) {
      glUniformMatrix4fv(shaderMRT_.getUniformLocation("model"), 1, GL_FALSE, meshes_[i]->getModel().data());
      glUniform3fv(shaderMRT_.getUniformLocation("diffuse"), 1,  meshes_[i]->getColor().data());
      meshes_[i]->display();
    }
    shaderMRT_.unbind();
    //Shadow Map
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboShadow_);
    glDrawBuffer(GL_NONE);
    glClear(GL_DEPTH_BUFFER_BIT);
    shaderShadow_.bind();
    glUniformMatrix4fv(shaderShadow_.getUniformLocation("view"), 1, GL_FALSE, light_->getView().data());
    glUniformMatrix4fv(shaderShadow_.getUniformLocation("projection"), 1, GL_FALSE, light_->getProjection().data());
    for (int i = 0; i < static_cast<int>(meshes_.size()); ++i) {
      glUniformMatrix4fv(shaderShadow_.getUniformLocation("model"), 1, GL_FALSE, meshes_[i]->getModel().data());
      meshes_[i]->display();
    }
    shaderShadow_.unbind();
    //Direct lighting
    glDisable(GL_DEPTH_TEST);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboDirect_);
    glDrawBuffers(1, drawBuffers);
    shaderDirect_.bind();
    Eigen::Matrix4f offsetMatrix;
    offsetMatrix << 0.5, 0.0, 0.0, 0.5,
                    0.0, 0.5, 0.0, 0.5,
                    0.0, 0.0, 0.5, 0.5,
                    0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4f lightMatrix = offsetMatrix*light_->getProjection()*light_->getView()*camera_->getInverseView();
    glUniformMatrix4fv(shaderDirect_.getUniformLocation("lightMatrix"), 1, GL_FALSE, lightMatrix.data());
    glUniformMatrix4fv(shaderDirect_.getUniformLocation("inverseProjection"), 1, GL_FALSE, camera_->getInverseProjection().data());
    glUniform1i(shaderDirect_.getUniformLocation("normalDepthTex"), 0);
    glUniform1i(shaderDirect_.getUniformLocation("diffuseTex"), 1);
    glUniform1i(shaderDirect_.getUniformLocation("randomTex"), 3);
    glUniform1i(shaderDirect_.getUniformLocation("shadowTex"), 4);
    Eigen::Vector4f light; light << light_->getPosition(), 1.0f;
    light = camera_->getView()*light;
    glUniform3fv(shaderDirect_.getUniformLocation("light"), 1,  light.data());
    quad_.display();
    shaderDirect_.unbind();
    //Indirect lighting
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboGI_);
    shaderGI_.bind();
    glUniformMatrix4fv(shaderGI_.getUniformLocation("inverseProjection"), 1, GL_FALSE, camera_->getInverseProjection().data());
    glUniform1i(shaderGI_.getUniformLocation("normalDepthTex"), 0);
    glUniform1i(shaderGI_.getUniformLocation("directTex"), 2);
    glUniform1i(shaderGI_.getUniformLocation("randomTex"), 3);
    quad_.display();
    shaderGI_.unbind();
    //Anti Aliasing
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    shaderFXAA_.bind();
    glUniform1i(shaderFXAA_.getUniformLocation("giTex"), 5);
    quad_.display();
    shaderFXAA_.unbind();
  };
 private:
  void textureInit(GLuint id, GLint param) {
      glBindTexture (GL_TEXTURE_2D, id);
      glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, param);
      glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, param);
  }

  void init() {
    glClearColor(0.0, 0.0, 0.0, 1.0);
//    glEnable(GL_MULTISAMPLE);
    // MRT FBO
    glGenFramebuffers(1, &fboMRT_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboMRT_);
    glGenTextures(2, texIds_);
    textureInit(texIds_[0], GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, texWidth_, texHeight_, 0, GL_RGBA, GL_FLOAT, NULL); //TODO AND OPTION
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, texWidth_, texHeight_, 0, GL_RGBA, GL_HALF_FLOAT, NULL);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texIds_[0], 0);
    textureInit(texIds_[1], GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth_, texHeight_, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, texIds_[1], 0);
    GLuint renderBufferId;
    glGenRenderbuffers(1, &renderBufferId);
    glBindRenderbuffer(GL_RENDERBUFFER, renderBufferId);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, texWidth_, texHeight_);
    glBindFramebuffer(GL_FRAMEBUFFER_EXT, fboMRT_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, renderBufferId);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    // Shadow Map FBO
    glGenFramebuffers(1, &fboShadow_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboShadow_);
    GLuint shadowTexID;
    glGenTextures(1, &shadowTexID);
    textureInit(shadowTexID, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, texWidth_, texHeight_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowTexID, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    // Direct FBO
    glGenFramebuffers(1, &fboDirect_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboDirect_);
    GLuint directTexID;
    glGenTextures(1, &directTexID);
    textureInit(directTexID, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth_, texHeight_, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, directTexID, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    // GI FBO
    glGenFramebuffers(1, &fboGI_);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fboGI_);
    GLuint giTexID;
    glGenTextures(1, &giTexID);
    textureInit(giTexID, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth_, texHeight_, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, giTexID, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    // Random texture for sampling
    GLuint randomTexID;
    glGenTextures(1, &randomTexID);
    textureInit(randomTexID, GL_REPEAT);
    glfwLoadTexture2D("random.tga", 0);
    // Bind texture for sampling
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texIds_[0]);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, texIds_[1]);
    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_2D, directTexID);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, randomTexID);
    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_2D, shadowTexID);
    glActiveTexture(GL_TEXTURE5);
    glBindTexture(GL_TEXTURE_2D, giTexID);
  }
  std::shared_ptr<Camera> light_;
  std::vector<std::shared_ptr<Object>> meshes_;
  std::shared_ptr<Camera> camera_;
  FullScreenQuad quad_;
  GLShader shaderMRT_, shaderShadow_, shaderDirect_, shaderGI_, shaderFXAA_;
  GLuint fboMRT_, fboDirect_, fboShadow_, fboGI_;
  GLuint texIds_[2];
  unsigned int texWidth_, texHeight_;
};
///////////////////////////////////////////////////////////////////////////////
#endif
///////////////////////////////////////////////////////////////////////////////
