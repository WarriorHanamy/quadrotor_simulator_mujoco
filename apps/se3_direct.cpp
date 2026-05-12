#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "quadrotor_sim/se3_controller.hpp"
#include "quadrotor_sim/shm/shm_layout.hpp"
#include "quadrotor_sim/shm/shm_backend.hpp"

using namespace quadrotor_sim;
using namespace quadrotor_sim::shm;
using namespace std::chrono;

namespace {

volatile sig_atomic_t g_exit = 0;

void sigint_handler(int /*sig*/) { g_exit = 1; }

struct Options {
  Se3Setpoint setpoint;
  double rate_hz = 500.0;
  std::string gains_file;
};

Options parse_args(int argc, char** argv) {
  Options opts;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "--pos-x" && i + 1 < argc)
      opts.setpoint.position[0] = std::stod(argv[++i]);
    else if (arg == "--pos-y" && i + 1 < argc)
      opts.setpoint.position[1] = std::stod(argv[++i]);
    else if (arg == "--pos-z" && i + 1 < argc)
      opts.setpoint.position[2] = std::stod(argv[++i]);
    else if (arg == "--yaw" && i + 1 < argc)
      opts.setpoint.yaw = std::stod(argv[++i]);
    else if (arg == "--rate" && i + 1 < argc)
      opts.rate_hz = std::stod(argv[++i]);
    else if (arg == "--gains-file" && i + 1 < argc)
      opts.gains_file = argv[++i];
  }
  return opts;
}

void wait_for_shm(StateWire*& state, ControlWire*& ctrl) {
  while (!g_exit) {
    state = OpenStateShm();
    ctrl  = OpenCtrlShm(/*write=*/true);
    if (state && ctrl) return;
    std::fprintf(stderr, "se3_direct: waiting for shared memory...\n");
    std::this_thread::sleep_for(milliseconds(500));
  }
}

}  // namespace

int main(int argc, char** argv) {
  Options opts = parse_args(argc, argv);

  std::signal(SIGINT, sigint_handler);
  std::signal(SIGTERM, sigint_handler);

  std::fprintf(stderr, "se3_direct: setpoint (%.2f, %.2f, %.2f), yaw=%.2f rad, rate=%.0f Hz\n",
              opts.setpoint.position[0], opts.setpoint.position[1],
              opts.setpoint.position[2], opts.setpoint.yaw, opts.rate_hz);

  Se3Controller controller(
      Se3Controller::LoadGainsFromYAML(opts.gains_file));

  StateWire*  shm_state = nullptr;
  ControlWire* shm_ctrl  = nullptr;
  wait_for_shm(shm_state, shm_ctrl);
  if (g_exit) return 0;

  auto loop_period = duration<double>(1.0 / opts.rate_hz);
  int iter = 0;
  int print_every = static_cast<int>(opts.rate_hz);

  std::fprintf(stderr, "se3_direct: running at %.0f Hz (Ctrl-C to stop)\n", opts.rate_hz);

  while (!g_exit) {
    auto loop_start = steady_clock::now();

    StateWire sw;
    if (!ReadConsistent(*shm_state, sw)) {
      std::this_thread::sleep_for(microseconds(100));
      continue;
    }

    State st;
    FromWire(sw, st);

    Control ctrl;
    controller.Compute(st, opts.setpoint, ctrl);

    ControlWire cw;
    ToWire(ctrl, cw);
    WriteBegin(shm_ctrl->sequence);
    cw.sequence = shm_ctrl->sequence;
    *shm_ctrl = cw;
    WriteEnd(shm_ctrl->sequence);

    iter++;
    if (iter >= print_every) {
      iter = 0;
      double e_p = std::sqrt(
          (st.position[0] - opts.setpoint.position[0]) * (st.position[0] - opts.setpoint.position[0]) +
          (st.position[1] - opts.setpoint.position[1]) * (st.position[1] - opts.setpoint.position[1]) +
          (st.position[2] - opts.setpoint.position[2]) * (st.position[2] - opts.setpoint.position[2]));
      std::fprintf(stderr, "se3_direct: t=%.2f pos=(%.3f,%.3f,%.3f) err=%.3f "
                  "thrust=%.2f T=(%+.3f,%+.3f,%+.3f)\n",
                  st.time, st.position[0], st.position[1], st.position[2],
                  e_p, ctrl.thrust,
                  ctrl.torque[0], ctrl.torque[1], ctrl.torque[2]);
    }

    auto elapsed = steady_clock::now() - loop_start;
    if (elapsed < loop_period) std::this_thread::sleep_for(loop_period - elapsed);
  }

  munmap(shm_state, sizeof(StateWire));
  munmap(shm_ctrl,  sizeof(ControlWire));
  std::fprintf(stderr, "se3_direct: shutdown\n");
  return 0;
}
