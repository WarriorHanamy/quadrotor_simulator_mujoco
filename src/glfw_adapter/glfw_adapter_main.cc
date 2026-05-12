/**
 * Render adapter for the quadrotor MuJoCo simulator.
 *
 * Reads QuadrotorState from /dev/shm/quadrotor_sim/state and renders
 * a MuJoCo visualization window. Does NOT run physics — purely a viewer.
 *
 * Mouse controls (MuJoCo native conventions):
 *   Left-drag          — orbit camera
 *   Shift+Left-drag    — rotate horizontally
 *   Right-drag         — pan
 *   Shift+Right-drag   — zoom
 *   Ctrl+Right-click   — select object
 *   Middle-drag        — zoom
 *   Scroll             — zoom
 *   Double-click       — center on body
 *
 * Keyboard shortcuts:
 *   Tab          — cycle camera type (free / tracking / fixed)
 *   [ / ]        — previous / next fixed camera
 *   R            — reset camera
 *   Space        — freeze / unfreeze display
 *   F1           — toggle contact points
 *   F2           — toggle contact forces
 *   F3           — toggle joints
 *   F4           — toggle actuators
 *   F5           — toggle transparency
 *   F6           — toggle center of mass
 *   F7           — toggle all visualization flags
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

// Mouse + keyboard state
int   mouse_action    = 0;
bool  mouse_button_down = false;
double mouse_last_x   = 0.0;
double mouse_last_y   = 0.0;
double last_click_time = 0.0;

bool paused           = false;

mjModel* g_model = nullptr;
mjData*  g_data  = nullptr;

void glfw_mouse_button(GLFWwindow* window, int button, int action, int mods) {
  if (action == GLFW_PRESS) {
    mouse_button_down = true;
    glfwGetCursorPos(window, &mouse_last_x, &mouse_last_y);

    // Double-click detection
    double now = glfwGetTime();
    if (button == GLFW_MOUSE_BUTTON_LEFT && (now - last_click_time) < 0.3) {
      // SELECT uses (reldx, reldy) as viewport pixel coordinates in MuJoCo 2.x
      double cx = mouse_last_x;
      double cy = mouse_last_y;
      mjv_moveCamera(g_model, mjMOUSE_SELECT, cx, cy, &scene, &cam);
      last_click_time = 0.0;
      return;
    }
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
      last_click_time = now;
    }

    // Assign action based on button + modifiers
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
      mouse_action = (mods & GLFW_MOD_SHIFT) ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
      if (mods & GLFW_MOD_CONTROL)
        mouse_action = mjMOUSE_SELECT;
      else if (mods & GLFW_MOD_SHIFT)
        mouse_action = mjMOUSE_ZOOM;
      else
        mouse_action = mjMOUSE_MOVE_V;  // pan via both MOVE_V and MOVE_H in cursor
    } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
      mouse_action = mjMOUSE_ZOOM;
    }
  } else if (action == GLFW_RELEASE) {
    mouse_button_down = false;
    mouse_action      = 0;
  }
}

void glfw_cursor_pos(GLFWwindow* /*window*/, double xpos, double ypos) {
  if (!mouse_button_down) {
    mouse_last_x = xpos;
    mouse_last_y = ypos;
    return;
  }

  double dx = xpos - mouse_last_x;
  double dy = ypos - mouse_last_y;
  mouse_last_x = xpos;
  mouse_last_y = ypos;

  if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) return;

  // Scale sensitivity per action type
  switch (mouse_action) {
    case mjMOUSE_ROTATE_V:
    case mjMOUSE_ROTATE_H:
      dx *= 0.012; dy *= 0.012;
      break;
    case mjMOUSE_MOVE_V:
      dx *= 0.02; dy *= 0.02;
      break;
    case mjMOUSE_ZOOM:
      dy *= 0.08;
      break;
  }

  if (mouse_action == mjMOUSE_MOVE_V) {
    mjv_moveCamera(g_model, mjMOUSE_MOVE_V, dx, dy, &scene, &cam);
    mjv_moveCamera(g_model, mjMOUSE_MOVE_H, dx, dy, &scene, &cam);
  } else {
    mjv_moveCamera(g_model, mouse_action, dx, dy, &scene, &cam);
  }
}

void glfw_scroll(GLFWwindow* /*window*/, double /*xoffset*/, double yoffset) {
  mjv_moveCamera(g_model, mjMOUSE_ZOOM, 0.0, -yoffset * 0.2, &scene, &cam);
}

void glfw_keyboard(GLFWwindow* /*window*/, int key, int /*scancode*/, int action, int /*mods*/) {
  if (action != GLFW_PRESS) return;

  int flag;
  switch (key) {
    case GLFW_KEY_TAB:
      // Cycle camera type: free → tracking → fixed → free
      if (cam.type == mjCAMERA_FREE) {
        cam.type = mjCAMERA_TRACKING;
      } else if (cam.type == mjCAMERA_TRACKING) {
        cam.type = mjCAMERA_FIXED;
        cam.fixedcamid = 0;
      } else {
        cam.type = mjCAMERA_FREE;
      }
      break;
    case GLFW_KEY_LEFT_BRACKET:
      if (cam.type == mjCAMERA_FIXED && g_model->ncam > 0)
        cam.fixedcamid = (cam.fixedcamid > 0) ? cam.fixedcamid - 1 : g_model->ncam - 1;
      break;
    case GLFW_KEY_RIGHT_BRACKET:
      if (cam.type == mjCAMERA_FIXED && g_model->ncam > 0)
        cam.fixedcamid = (cam.fixedcamid + 1) % g_model->ncam;
      break;
    case GLFW_KEY_R:
      mjv_defaultCamera(&cam);
      mjv_defaultOption(&vopt);
      cam.type       = mjCAMERA_FREE;
      cam.lookat[0]  = 0.0;
      cam.lookat[1]  = 0.0;
      cam.lookat[2]  = 1.0;
      cam.distance   = 5.0;
      cam.azimuth    = 0.0;
      cam.elevation  = -20.0;
      break;
    case GLFW_KEY_SPACE:
      paused = !paused;
      break;
    case GLFW_KEY_F1:
      flag = mjVIS_CONTACTPOINT;
      vopt.flags[flag] = !vopt.flags[flag];
      break;
    case GLFW_KEY_F2:
      flag = mjVIS_CONTACTFORCE;
      vopt.flags[flag] = !vopt.flags[flag];
      break;
    case GLFW_KEY_F3:
      flag = mjVIS_JOINT;
      vopt.flags[flag] = !vopt.flags[flag];
      break;
    case GLFW_KEY_F4:
      flag = mjVIS_ACTUATOR;
      vopt.flags[flag] = !vopt.flags[flag];
      break;
    case GLFW_KEY_F5:
      flag = mjVIS_TRANSPARENT;
      vopt.flags[flag] = !vopt.flags[flag];
      break;
    case GLFW_KEY_F6:
      flag = mjVIS_COM;
      vopt.flags[flag] = !vopt.flags[flag];
      break;
    case GLFW_KEY_F7:
      // Toggle all commonly-used visualization flags
      for (int i = 0; i < mjNVISFLAG; i++)
        vopt.flags[i] = !vopt.flags[i];
      break;
    default:
      break;
  }
}
}  // namespace

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

  // Store model and data for callbacks
  g_model = m;
  g_data  = d;
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

  // Set up free camera with reasonable initial view
  cam.type = mjCAMERA_FREE;
  cam.lookat[0] = 0.0;
  cam.lookat[1] = 0.0;
  cam.lookat[2] = 1.0;
  cam.distance  = 5.0;
  cam.azimuth   = 0.0;
  cam.elevation = -20.0;

  // Register mouse and keyboard callbacks
  glfwSetMouseButtonCallback(window, glfw_mouse_button);
  glfwSetCursorPosCallback(window, glfw_cursor_pos);
  glfwSetScrollCallback(window, glfw_scroll);
  glfwSetKeyCallback(window, glfw_keyboard);

  std::printf("glfw_adapter: viewer ready\n");

  // Render loop
  while (!exit_request.load() && !glfwWindowShouldClose(window)) {
    // Read latest state from shared memory
    if (!paused) {
      QuadrotorState state;
      if (shm_read(*shm_state, state)) {
        d->qpos[0] = state.position[0];
        d->qpos[1] = state.position[1];
        d->qpos[2] = state.position[2];
        d->qpos[3] = state.orientation[0];
        d->qpos[4] = state.orientation[1];
        d->qpos[5] = state.orientation[2];
        d->qpos[6] = state.orientation[3];

        mj_forward(m, d);
      }
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
