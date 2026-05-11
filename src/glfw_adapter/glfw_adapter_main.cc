/**
 * GLFW render adapter for the quadrotor MuJoCo simulator.
 *
 * Reads QuadrotorState from /dev/shm/quadrotor_sim/state and renders
 * a MuJoCo visualization window. Does NOT run physics — purely a viewer.
 */

#include <cstdio>
#include <csignal>
#include <atomic>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "sim_schema.h"
#include "shm_backend.h"

namespace {
std::atomic<bool> exit_request{false};

void sigint_handler(int /*sig*/) {
  exit_request.store(true);
}

// MuJoCo rendering structures
mjvScene scene;
mjvCamera cam;
mjvOption vopt;
mjvPerturb pert;
mjrContext con;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: quadrotor_sim_glfw_adapter <model.xml>\n");
    return 1;
  }
  std::string model_path = argv[1];

  std::signal(SIGINT,  sigint_handler);
  std::signal(SIGTERM, sigint_handler);

  // Load model (for geometry, not physics)
  char loadError[1024] = "";
  mjModel* m = mj_loadXML(model_path.c_str(), nullptr, loadError, 1024);
  if (!m) {
    std::fprintf(stderr, "glfw_adapter: cannot load model: %s\n", loadError);
    return 1;
  }
  mjData* d = mj_makeData(m);
  if (!d) {
    mj_deleteModel(m);
    return 1;
  }

  // Attach to shared memory
  QuadrotorState* shm_state = open_state_shm();
  if (!shm_state) {
    std::fprintf(stderr, "glfw_adapter: cannot open state shm. Is quadrotor_sim_core running?\n");
    mj_deleteData(d);
    mj_deleteModel(m);
    return 1;
  }

  // Initialize GLFW
  if (!glfwInit()) {
    std::fprintf(stderr, "glfw_adapter: glfwInit failed\n");
    return 1;
  }

  // Create window
  GLFWwindow* window = glfwCreateWindow(1280, 720, "Quadrotor Sim (Viewer)", nullptr, nullptr);
  if (!window) {
    std::fprintf(stderr, "glfw_adapter: glfwCreateWindow failed\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // Initialize MuJoCo rendering
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&vopt);
  mjv_defaultPerturb(&pert);
  mjv_defaultScene(&scene);
  mjr_defaultContext(&con);

  mjv_makeScene(m, &scene, 1000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // Set camera from model (use bottom_camera if available)
  if (m->ncam > 0) {
    // Find camera named "bottom_camera" or use first
    int cam_id = 0;
    for (int i = 0; i < m->ncam; i++) {
      if (std::string(m->names + m->name_camadr[i]) == "bottom_camera") {
        cam_id = i;
        break;
      }
    }
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = cam_id;
  }

  std::printf("glfw_adapter: viewer ready\n");

  // Render loop
  while (!exit_request.load() && !glfwWindowShouldClose(window)) {
    // Read latest state from shared memory
    QuadrotorState state;
    if (shm_read(*shm_state, state)) {
      // Update qpos from state
      d->qpos[0] = state.position[0];
      d->qpos[1] = state.position[1];
      d->qpos[2] = state.position[2];
      d->qpos[3] = state.orientation[0];
      d->qpos[4] = state.orientation[1];
      d->qpos[5] = state.orientation[2];
      d->qpos[6] = state.orientation[3];

      mj_forward(m, d);
    }

    // Get framebuffer size
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // Update scene and render
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();

    // Limit render rate
    std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 FPS
  }

  // Cleanup
  mjr_freeContext(&con);
  mjv_freeScene(&scene);
  munmap(shm_state, sizeof(QuadrotorState));
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}
