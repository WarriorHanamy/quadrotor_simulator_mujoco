#include <cstdio>
#include <csignal>
#include <string>
#include <memory>

#include "sim_core.h"

namespace {
std::unique_ptr<QuadrotorSimCore> g_core;

void sigint_handler(int /*sig*/) {
  if (g_core) g_core->request_shutdown();
}
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: quadrotor_sim_core <model.xml> [--real-time-factor R] "
                         "[--ctrlnoise-std S] [--ctrlnoise-rate R]\n");
    return 1;
  }

  std::string model_path = argv[1];
  QuadrotorSimCore::Params params;

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

  g_core = std::make_unique<QuadrotorSimCore>(model_path, params);
  if (!g_core->valid()) {
    std::fprintf(stderr, "sim_core: failed to initialize\n");
    return 1;
  }

  std::printf("quadrotor_sim_core: MuJoCo version %s\n", mj_versionString());
  std::printf("quadrotor_sim_core: model %s, real_time_factor=%.2f\n",
              model_path.c_str(), params.real_time_factor);

  g_core->run();
  g_core.reset();

  std::printf("quadrotor_sim_core: shutdown complete\n");
  return 0;
}
