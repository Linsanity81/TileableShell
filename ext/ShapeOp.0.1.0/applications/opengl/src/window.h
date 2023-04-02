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
#ifndef WINDOW_H
#define WINDOW_H
///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <cassert>
#include <string>
#include <memory>
#include <functional>
#include "gldebug.h"
#include "camera.h"
#include "mesh.h"
///////////////////////////////////////////////////////////////////////////////
void resize(int width, int height) {
}
void keyboard(int key, int action) {
  if (action == GLFW_PRESS) {
    switch (key) {
      case GLFW_KEY_UP: {
      } break;
      case GLFW_KEY_DOWN: {
      } break;
      case GLFW_KEY_LEFT: {
      } break;
      case GLFW_KEY_RIGHT: {
      } break;
      case GLFW_KEY_ESC:
        glfwCloseWindow();
        break;
      case GLFW_KEY_SPACE:
        break;
    }
  }
}
void mouse_button(int button, int action) {
}
void mouse_move(int x, int y, int old_x, int old_y) {
}
void mouse_pos(int x, int y) {
}
void mouse_wheel(int pos) {
}
///////////////////////////////////////////////////////////////////////////////
/** \brief GLWindow. A OpenGL window to draw and simulate a list of ShapeOpSurfaceMeshes.*/
class GLWindow {
 public:
  GLWindow(const std::string &title, int width, int height) : title_(title), width_(width), height_(height) {
    create();
    lastTimeFPS_ = glfwGetTime();
    lastTimeUI_ = glfwGetTime();
    glfwGetMousePos(&lastPosX_, &lastPosY_);
    nbFrames_ = 0;
    mesh_ = 0;
  }
  void loop(const std::function<void()> &display,
            std::vector<std::shared_ptr<ShapeOpSurfaceMesh>>& s,
            const std::shared_ptr<Camera> &c) {
    while (glfwGetKey(GLFW_KEY_ESC) != GLFW_PRESS && glfwGetWindowParam(GLFW_OPENED)) {
      camera(c);
      mesh(s[mesh_]);
      s[mesh_]->process();
      //double t1 = glfwGetTime();
      display();
      GLCHECK
      glfwSwapBuffers();
      //std::cout << "display: " << (glfwGetTime()-t1)*1000.0 << " ms" << std::endl;
      showFPS();
    }
    glfwTerminate();
  }
 private:
  void mesh(std::shared_ptr<ShapeOpSurfaceMesh>& s) {
    if(glfwGetKey( 87 ) == GLFW_PRESS) {
//        std::cout << "Disable Force" << std::endl;
        s->enableForce(-1);
    }
    if(glfwGetKey( 81 ) == GLFW_PRESS) {
//        std::cout << "Enable Force" << std::endl;
        s->enableForce(2045);
    }
    if(glfwGetKey( 69 ) == GLFW_PRESS) {
//        std::cout << "Enable Force" << std::endl;
        s->enableForce(5503);
    }
    if(glfwGetKey( 83 ) == GLFW_PRESS) {
//        std::cout << "Enable Force" << std::endl;
        s->enableForce(6668);
    }

    if(glfwGetKey( 90 ) == GLFW_PRESS) {
//        std::cout << "Enable Force" << std::endl;
        s->rotate(0.01*M_PI);
    }
    if(glfwGetKey( 88 ) == GLFW_PRESS) {
//        std::cout << "Enable Force" << std::endl;
        s->rotate(-0.01*M_PI);
    }
    for(int i = 1; i <= 5; ++i) {
        if(glfwGetKey(48 + i) == GLFW_PRESS) {
            mesh_ = i-1;
//            std::cout << "Mesh: " << mesh_ << std::endl;
        }
    }
  }

  void camera(const std::shared_ptr<Camera> &c) {
    double deltaTime = glfwGetTime() - lastTimeUI_;
    float speedR = 0.1f * deltaTime;
    float speedT = 1.0f * deltaTime;
    // Rotate
    if (glfwGetMouseButton(0) == GLFW_PRESS) {
      int posX, posY;
      glfwGetMousePos(&posX, &posY);
      float dx = speedR * static_cast<float>(lastPosX_ - posX);
      float dy = speedR * static_cast<float>(lastPosY_ - posY);
      c->rotateUp(dx);
      c->rotateStraff(dy);
      c->updateView();
    }
    if (glfwGetMouseButton(1) == GLFW_PRESS) {

    }
    // Move forward
    if (glfwGetKey( GLFW_KEY_UP ) == GLFW_PRESS) {
      c->move(speedT);
      c->updateView();
    }
    // Move backward
    if (glfwGetKey( GLFW_KEY_DOWN ) == GLFW_PRESS) {
      c->move(-speedT);
      c->updateView();
    }
    // Strafe right
    if (glfwGetKey( GLFW_KEY_RIGHT ) == GLFW_PRESS) {
      c->straff(speedT);
      c->updateView();
    }
    // Strafe left
    if (glfwGetKey( GLFW_KEY_LEFT ) == GLFW_PRESS) {
      c->straff(-speedT);
      c->updateView();
    }
    lastTimeUI_ = glfwGetTime();
    glfwGetMousePos(&lastPosX_, &lastPosY_);
  }
  void showFPS() {
    // Measure speed
    double currentTime = glfwGetTime();
    nbFrames_++;
    if ( currentTime - lastTimeFPS_ >= 1.0 ) {
      std::cout << nbFrames_ << "fps " << 1000.0 / double(nbFrames_) << "ms" << std::endl;
      nbFrames_ = 0;
      lastTimeFPS_ = currentTime;
    }
  }
  void create() {
    if (!glfwInit()) return;
    glfwOpenWindowHint(GLFW_OPENGL_VERSION_MAJOR, 3);
    glfwOpenWindowHint(GLFW_OPENGL_VERSION_MINOR, 2);
    glfwOpenWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//    glfwOpenWindowHint(GLFW_FSAA_SAMPLES, 4);
    if (!glfwOpenWindow(width_, height_, 0, 0, 0, 0, 32, 0, /*GLFW_FULLSCREEN*/GLFW_WINDOW)) {
      glfwTerminate();
      return;
    }
    glewExperimental = true;
    if (glewInit() != GLEW_NO_ERROR) {
      glfwTerminate();
      return;
    }
    glfwSetWindowTitle(title_.c_str());
//    glfwSwapInterval(0);
    return;
  }
  int nbFrames_;
  double lastTimeFPS_;
  double lastTimeUI_;
  int lastPosX_, lastPosY_;
  std::string title_;
  int width_;
  int height_;
  int mesh_;
};
///////////////////////////////////////////////////////////////////////////////
#endif
///////////////////////////////////////////////////////////////////////////////
