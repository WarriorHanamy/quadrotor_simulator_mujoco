#include "quadrotor_sim/mujoco/render.hpp"

#include <cstdio>
#include <cstring>

namespace quadrotor_sim::mujoco {

Renderer::Renderer(const std::string& model_xml, int width, int height)
    : width_(width), height_(height) {
  char err[1024] = "";
  m_ = mj_loadXML(model_xml.c_str(), nullptr, err, 1024);
  if (!m_) { std::fprintf(stderr, "render: load failed: %s\n", err); return; }
  d_ = mj_makeData(m_);
  if (!d_) { mj_deleteModel(m_); m_ = nullptr; return; }

  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&vopt_);
  mjv_defaultPerturb(&pert_);
  mjv_defaultScene(&scene_);

  mjv_makeScene(m_, &scene_, 1000);
  mjr_defaultContext(&con_);
  mjr_makeContext(m_, &con_, mjFONTSCALE_150);

  cam_.type = mjCAMERA_FREE;
  cam_.lookat[0] = 0.0; cam_.lookat[1] = 0.0; cam_.lookat[2] = 1.0;
  cam_.distance  = 5.0; cam_.azimuth = 0.0; cam_.elevation = -20.0;

  pixels_.resize(width_ * height_ * 3);
}

Renderer::~Renderer() {
  mjr_freeContext(&con_);
  mjv_freeScene(&scene_);
  if (d_) mj_deleteData(d_);
  if (m_) mj_deleteModel(m_);
}

const std::vector<uint8_t>& Renderer::Render(const State& state) {
  if (!m_ || !d_) return pixels_;

  d_->qpos[0] = state.position[0];
  d_->qpos[1] = state.position[1];
  d_->qpos[2] = state.position[2];
  d_->qpos[3] = state.orientation[0];
  d_->qpos[4] = state.orientation[1];
  d_->qpos[5] = state.orientation[2];
  d_->qpos[6] = state.orientation[3];

  mj_forward(m_, d_);

  mjrRect viewport = {0, 0, width_, height_};
  mjv_updateScene(m_, d_, &vopt_, &pert_, &cam_, mjCAT_ALL, &scene_);
  mjr_render(viewport, &scene_, &con_);

  mjr_readPixels(pixels_.data(), nullptr, viewport, &con_);

  return pixels_;
}

}  // namespace quadrotor_sim::mujoco
