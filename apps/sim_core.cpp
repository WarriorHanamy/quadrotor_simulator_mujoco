#include <cstdio>
#include <csignal>
#include <string>
#include <memory>

#include "quadrotor_sim/mujoco/sim_core.hpp"

namespace {
std::unique_ptr<quadrotor_sim::mujoco::SimCore> g_core;

void sigint_handler(int /*sig*/) {
  if (g_core) g_core->RequestShutdown();
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: quadrotor_sim_core <model.xml> [--real-time-factor R] "
                         "[--ctrlnoise-std S] [--ctrlnoise-rate R]\n");
    return 1;
  }

  std::string model_path = argv[1];
  quadrotor_sim::mujoco::SimCore::Params params;

  for (int i = 2; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--real-time-factor" && i + 1 < argc) {
      params.real_time_factor = std::stod(argv[++i]);
    } else if (arg == "--ctrlnoise-std" && i + 1 < argc) {
      params.ctrlnoise_std = std::stod(argv[++i]);
    } else if (arg == "--ctrlnoise-rate" && i + 1 < argc) {
      params.ctrlnoise_rate = std::stod(argv[++i]);
    }
  }

  std::signal(SIGINT, sigint_handler);
  std::signal(SIGTERM, sigint_handler);

  g_core = std::make_unique<quadrotor_sim::mujoco::SimCore>(model_path, params);
  if (!g_core->valid()) {
    std::fprintf(stderr, "sim_core: failed to initialize\n");
    return 1;
  }

  std::fprintf(stderr, "quadrotor_sim_core: MuJoCo %s, model %s, rtf=%.2f\n",
              mj_versionString(), model_path.c_str(), params.real_time_factor);

  g_core->Run();
  std::fprintf(stderr, "quadrotor_sim_core: shutdown complete\n");
  g_core.reset();

  std::fprintf(stderr, "quadrotor_sim_core: shutdown complete\n");
  return 0;
}
