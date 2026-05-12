#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "quadrotor_sim/types.hpp"

namespace quadrotor_sim::mujoco {

/**
 * Offscreen MuJoCo renderer.
 *
 * Takes a quadrotor State and renders it to an RGB pixel buffer.
 * Zero GLFW dependency — works in headless environments.
 */
class Renderer {
public:
  Renderer(const std::string& model_xml, int width = 1280, int height = 720);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  /** Render the scene from current state. Returns RGB pixel buffer (row-major, rgb8). */
  const std::vector<uint8_t>& Render(const State& state);

  int width()  const { return width_; }
  int height() const { return height_; }

private:
  mjModel* m_ = nullptr;
  mjData*  d_ = nullptr;
  mjvScene scene_{};
  mjvCamera cam_{};
  mjvOption vopt_{};
  mjvPerturb pert_{};
  mjrContext con_{};

  int width_  = 1280;
  int height_ = 720;
  std::vector<uint8_t> pixels_;
};

}  // namespace quadrotor_sim::mujoco
