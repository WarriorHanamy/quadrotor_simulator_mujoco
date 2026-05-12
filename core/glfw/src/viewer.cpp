#include "quadrotor_sim/glfw/viewer.hpp"

#include <cstdio>
#include <chrono>
#include <thread>

#include <GLFW/glfw3.h>

namespace quadrotor_sim::glfw {

struct Viewer::Impl {
  GLFWwindow* window = nullptr;
  int width  = 0;
  int height = 0;
  GLuint texture = 0;

  static void MouseButtonCb(GLFWwindow* w, int btn, int act, int /*mods*/) {
    if (act == GLFW_PRESS) {
      double x, y;
      glfwGetCursorPos(w, &x, &y);
    }
  }
  static void CursorPosCb(GLFWwindow* /*w*/, double /*x*/, double /*y*/) {}
  static void ScrollCb(GLFWwindow* /*w*/, double /*xoff*/, double /*yoff*/) {}
};

Viewer::Viewer(const char* title, int width, int height) {
  if (!glfwInit()) { std::fprintf(stderr, "viewer: glfwInit failed\n"); return; }

  impl_ = new Impl();
  impl_->width  = width;
  impl_->height = height;

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

  impl_->window = glfwCreateWindow(width, height, title, nullptr, nullptr);
  if (!impl_->window) {
    std::fprintf(stderr, "viewer: window creation failed\n");
    glfwTerminate();
    delete impl_; impl_ = nullptr;
    return;
  }
  glfwMakeContextCurrent(impl_->window);
  glfwSwapInterval(1);

  glfwSetWindowUserPointer(impl_->window, impl_);
  glfwSetMouseButtonCallback(impl_->window, Impl::MouseButtonCb);
  glfwSetCursorPosCallback(impl_->window, Impl::CursorPosCb);
  glfwSetScrollCallback(impl_->window, Impl::ScrollCb);

  glGenTextures(1, &impl_->texture);
  glBindTexture(GL_TEXTURE_2D, impl_->texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
}

Viewer::~Viewer() {
  if (impl_) {
    if (impl_->texture) glDeleteTextures(1, &impl_->texture);
    if (impl_->window) glfwDestroyWindow(impl_->window);
    delete impl_;
  }
  glfwTerminate();
}

bool Viewer::ShouldClose() const {
  return !impl_ || glfwWindowShouldClose(impl_->window);
}

void Viewer::RequestClose() {
  if (impl_) glfwSetWindowShouldClose(impl_->window, GLFW_TRUE);
}

void Viewer::Present(const std::vector<uint8_t>& pixels, int w, int h) {
  if (!impl_ || !impl_->window) return;

  glfwMakeContextCurrent(impl_->window);

  glBindTexture(GL_TEXTURE_2D, impl_->texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

  glViewport(0, 0, impl_->width, impl_->height);
  glClear(GL_COLOR_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-1, 1, -1, 1, -1, 1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, impl_->texture);
  glBegin(GL_QUADS);
  glTexCoord2f(0, 1); glVertex2f(-1, -1);
  glTexCoord2f(1, 1); glVertex2f( 1, -1);
  glTexCoord2f(1, 0); glVertex2f( 1,  1);
  glTexCoord2f(0, 0); glVertex2f(-1,  1);
  glEnd();
  glDisable(GL_TEXTURE_2D);

  glfwSwapBuffers(impl_->window);
}

void Viewer::PollEvents() {
  if (impl_) glfwPollEvents();
}

}  // namespace quadrotor_sim::glfw
