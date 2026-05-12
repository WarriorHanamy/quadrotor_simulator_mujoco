/**
 * Direct shared-memory backend for the SE(3) controller.
 *
 * Opens /dev/shm/quadrotor_sim/state (read) and /dev/shm/quadrotor_sim/ctrl
 * (write), runs the Se3Controller in a fixed-rate loop.
 *
 * Zero MuJoCo / ROS dependency — pure shm I/O.
 */

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "se3_controller.h"
#include "shm_backend.h"

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
    if (arg == "--pos-x" && i + 1 < argc) {
      opts.setpoint.position[0] = std::stod(argv[++i]);
    } else if (arg == "--pos-y" && i + 1 < argc) {
      opts.setpoint.position[1] = std::stod(argv[++i]);
    } else if (arg == "--pos-z" && i + 1 < argc) {
      opts.setpoint.position[2] = std::stod(argv[++i]);
    } else if (arg == "--yaw" && i + 1 < argc) {
      opts.setpoint.yaw = std::stod(argv[++i]);
    } else if (arg == "--rate" && i + 1 < argc) {
      opts.rate_hz = std::stod(argv[++i]);
    } else if (arg == "--gains-file" && i + 1 < argc) {
      opts.gains_file = argv[++i];
    }
  }
  return opts;
}

void wait_for_shm(QuadrotorState*& shm_state, QuadrotorControl*& shm_ctrl) {
  while (!g_exit) {
    shm_state = open_state_shm();
    shm_ctrl  = open_ctrl_shm(/*write=*/true);
    if (shm_state && shm_ctrl) return;
    std::fprintf(stderr, "se3_direct: waiting for shared memory "
                          "(is quadrotor_sim_core running?)...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

}  // namespace

int main(int argc, char** argv) {
  Options opts = parse_args(argc, argv);

  std::signal(SIGINT, sigint_handler);
  std::signal(SIGTERM, sigint_handler);

  std::printf("se3_direct: setpoint (%.2f, %.2f, %.2f), yaw=%.2f rad, rate=%.0f Hz\n",
              opts.setpoint.position[0], opts.setpoint.position[1],
              opts.setpoint.position[2], opts.setpoint.yaw, opts.rate_hz);

  Se3Controller controller(
      Se3Controller::LoadGainsFromYAML(opts.gains_file));

  QuadrotorState*  shm_state = nullptr;
  QuadrotorControl* shm_ctrl  = nullptr;
  wait_for_shm(shm_state, shm_ctrl);
  if (g_exit) return 0;

  auto loop_period = std::chrono::duration<double>(1.0 / opts.rate_hz);

  int    iter       = 0;
  int    print_every = static_cast<int>(opts.rate_hz);  // print once per second

  std::printf("se3_direct: running at %.0f Hz (Ctrl-C to stop)\n", opts.rate_hz);

  while (!g_exit) {
    auto loop_start = std::chrono::steady_clock::now();

    // Read current state with seqlock
    QuadrotorState state;
    if (!shm_read(*shm_state, state)) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
      continue;
    }

    // Compute control
    QuadrotorControl ctrl;
    controller.compute(state, opts.setpoint, ctrl);

    // Write control with seqlock
    ctrl.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    shm_write_begin(shm_ctrl->sequence);
    *shm_ctrl = ctrl;
    shm_write_end(shm_ctrl->sequence);

    // Periodic status
    iter++;
    if (iter >= print_every) {
      iter = 0;
      double e_p = std::sqrt(
          (state.position[0] - opts.setpoint.position[0]) *
              (state.position[0] - opts.setpoint.position[0]) +
          (state.position[1] - opts.setpoint.position[1]) *
              (state.position[1] - opts.setpoint.position[1]) +
          (state.position[2] - opts.setpoint.position[2]) *
              (state.position[2] - opts.setpoint.position[2]));
      std::printf("se3_direct: t=%.2f pos=(%.3f,%.3f,%.3f) err=%.3f "
                  "thrust=%.2f τ=(%+.3f,%+.3f,%+.3f)\n",
                  state.time,
                  state.position[0], state.position[1], state.position[2],
                  e_p, ctrl.thrust,
                  ctrl.torque[0], ctrl.torque[1], ctrl.torque[2]);
    }

    // Maintain fixed rate
    auto elapsed = std::chrono::steady_clock::now() - loop_start;
    if (elapsed < loop_period) {
      std::this_thread::sleep_for(loop_period - elapsed);
    }
  }

  if (shm_state) munmap(shm_state, sizeof(QuadrotorState));
  if (shm_ctrl)  munmap(shm_ctrl,  sizeof(QuadrotorControl));
  std::printf("se3_direct: shutdown\n");
  return 0;
}
