#pragma once

#include <atomic>
#include <cstdint>
#include <vector>

namespace quadrotor_sim::glfw {

/**
 * GLFW window wrapper for displaying rendered frames.
 *
 * Creates a window, presents RGB pixel data each frame, and handles
 * mouse/keyboard input via MuJoCo's camera movement callbacks.
 */
class Viewer {
public:
  Viewer(const char* title, int width, int height);
  ~Viewer();

  Viewer(const Viewer&) = delete;
  Viewer& operator=(const Viewer&) = delete;

  /** True while window is open and no exit requested. */
  bool ShouldClose() const;
  void RequestClose();

  /** Present an RGB8 pixel buffer (row-major, width*height*3). */
  void Present(const std::vector<uint8_t>& pixels, int width, int height);

  /** Poll events. Call each frame. */
  void PollEvents();

private:
  struct Impl;
  Impl* impl_ = nullptr;
};

}  // namespace quadrotor_sim::glfw
