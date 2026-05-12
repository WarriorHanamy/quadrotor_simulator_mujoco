#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "quadrotor_sim/shm/shm_layout.hpp"
#include "quadrotor_sim/shm/shm_backend.hpp"

using namespace quadrotor_sim::shm;
using namespace std::chrono;

namespace {

std::atomic<bool> exit_request{false};
void sigint_handler(int /*sig*/) { exit_request.store(true); }

mjModel*    g_m  = nullptr;
mjData*     g_d  = nullptr;
mjvScene    scene;
mjvCamera   cam;
mjvOption   vopt;
mjvPerturb  pert;
mjrContext  con;

bool paused           = false;
int   mouse_action     = 0;
bool  mouse_button_down = false;
double mouse_last_x     = 0.0;
double mouse_last_y     = 0.0;
double last_click_time  = 0.0;

void glfw_mouse_button(GLFWwindow* window, int button, int action, int mods) {
  if (action == GLFW_PRESS) {
    mouse_button_down = true;
    glfwGetCursorPos(window, &mouse_last_x, &mouse_last_y);
    double now = glfwGetTime();
    if (button == GLFW_MOUSE_BUTTON_LEFT && (now - last_click_time) < 0.3) {
      double cx = mouse_last_x, cy = mouse_last_y;
      mjv_moveCamera(g_m, mjMOUSE_SELECT, cx, cy, &scene, &cam);
      last_click_time = 0.0;
      return;
    }
    if (button == GLFW_MOUSE_BUTTON_LEFT) { last_click_time = now; }
    if (button == GLFW_MOUSE_BUTTON_LEFT)
      mouse_action = (mods & GLFW_MOD_SHIFT) ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
      if (mods & GLFW_MOD_CONTROL) mouse_action = mjMOUSE_SELECT;
      else if (mods & GLFW_MOD_SHIFT) mouse_action = mjMOUSE_ZOOM;
      else mouse_action = mjMOUSE_MOVE_V;
    } else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
      mouse_action = mjMOUSE_ZOOM;
  } else if (action == GLFW_RELEASE) {
    mouse_button_down = false;
    mouse_action = 0;
  }
}

void glfw_cursor_pos(GLFWwindow* /*window*/, double xpos, double ypos) {
  if (!mouse_button_down) { mouse_last_x = xpos; mouse_last_y = ypos; return; }
  double dx = xpos - mouse_last_x, dy = ypos - mouse_last_y;
  mouse_last_x = xpos; mouse_last_y = ypos;
  if (std::abs(dx) < 1e-6 && std::abs(dy) < 1e-6) return;
  switch (mouse_action) {
    case mjMOUSE_ROTATE_V: case mjMOUSE_ROTATE_H: dx *= 0.012; dy *= 0.012; break;
    case mjMOUSE_MOVE_V: dx *= 0.02; dy *= 0.02; break;
    case mjMOUSE_ZOOM: dy *= 0.08; break;
  }
  if (mouse_action == mjMOUSE_MOVE_V) {
    mjv_moveCamera(g_m, mjMOUSE_MOVE_V, dx, dy, &scene, &cam);
    mjv_moveCamera(g_m, mjMOUSE_MOVE_H, dx, dy, &scene, &cam);
  } else {
    mjv_moveCamera(g_m, mouse_action, dx, dy, &scene, &cam);
  }
}

void glfw_scroll(GLFWwindow* /*window*/, double /*xoff*/, double yoff) {
  mjv_moveCamera(g_m, mjMOUSE_ZOOM, 0.0, -yoff * 0.2, &scene, &cam);
}

void glfw_keyboard(GLFWwindow* /*w*/, int key, int /*sc*/, int act, int /*mods*/) {
  if (act != GLFW_PRESS) return;
  int flag;
  switch (key) {
    case GLFW_KEY_TAB:
      if (cam.type == mjCAMERA_FREE) cam.type = mjCAMERA_TRACKING;
      else if (cam.type == mjCAMERA_TRACKING) { cam.type = mjCAMERA_FIXED; cam.fixedcamid = 0; }
      else cam.type = mjCAMERA_FREE;
      break;
    case GLFW_KEY_LEFT_BRACKET:
      if (cam.type == mjCAMERA_FIXED && g_m->ncam > 0)
        cam.fixedcamid = (cam.fixedcamid > 0) ? cam.fixedcamid - 1 : g_m->ncam - 1;
      break;
    case GLFW_KEY_RIGHT_BRACKET:
      if (cam.type == mjCAMERA_FIXED && g_m->ncam > 0)
        cam.fixedcamid = (cam.fixedcamid + 1) % g_m->ncam;
      break;
    case GLFW_KEY_R:
      mjv_defaultCamera(&cam); mjv_defaultOption(&vopt);
      cam.type = mjCAMERA_FREE;
      cam.lookat[0] = 0.0; cam.lookat[1] = 0.0; cam.lookat[2] = 1.0;
      cam.distance = 5.0; cam.azimuth = 0.0; cam.elevation = -20.0;
      break;
    case GLFW_KEY_SPACE: paused = !paused; break;
    case GLFW_KEY_F1: flag = mjVIS_CONTACTPOINT;  vopt.flags[flag] = !vopt.flags[flag]; break;
    case GLFW_KEY_F2: flag = mjVIS_CONTACTFORCE;  vopt.flags[flag] = !vopt.flags[flag]; break;
    case GLFW_KEY_F3: flag = mjVIS_JOINT;         vopt.flags[flag] = !vopt.flags[flag]; break;
    case GLFW_KEY_F4: flag = mjVIS_ACTUATOR;      vopt.flags[flag] = !vopt.flags[flag]; break;
    case GLFW_KEY_F5: flag = mjVIS_TRANSPARENT;   vopt.flags[flag] = !vopt.flags[flag]; break;
    case GLFW_KEY_F6: flag = mjVIS_COM;           vopt.flags[flag] = !vopt.flags[flag]; break;
    case GLFW_KEY_F7:
      for (int i = 0; i < mjNVISFLAG; i++) vopt.flags[i] = !vopt.flags[i];
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

  std::signal(SIGINT, sigint_handler);
  std::signal(SIGTERM, sigint_handler);

  char loadError[1024] = "";
  mjModel* m = mj_loadXML(model_path.c_str(), nullptr, loadError, 1024);
  if (!m) { std::fprintf(stderr, "glfw_adapter: cannot load model: %s\n", loadError); return 1; }
  mjData* d = mj_makeData(m);
  if (!d) { mj_deleteModel(m); return 1; }

  StateWire* shm_state = OpenStateShm();
  g_m = m; g_d = d;
  if (!shm_state) {
    std::fprintf(stderr, "glfw_adapter: cannot open state shm. Is sim_core running?\n");
    mj_deleteData(d); mj_deleteModel(m); return 1;
  }

  if (!glfwInit()) { std::fprintf(stderr, "glfw_adapter: glfwInit failed\n"); return 1; }
  GLFWwindow* window = glfwCreateWindow(1280, 720, "Quadrotor Sim (Viewer)", nullptr, nullptr);
  if (!window) { std::fprintf(stderr, "glfw_adapter: window creation failed\n"); glfwTerminate(); return 1; }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjv_defaultCamera(&cam);
  mjv_defaultOption(&vopt);
  mjv_defaultPerturb(&pert);
  mjv_defaultScene(&scene);
  mjr_defaultContext(&con);
  mjv_makeScene(m, &scene, 1000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  cam.type = mjCAMERA_FREE;
  cam.lookat[0] = 0.0; cam.lookat[1] = 0.0; cam.lookat[2] = 1.0;
  cam.distance = 5.0; cam.azimuth = 0.0; cam.elevation = -20.0;

  glfwSetMouseButtonCallback(window, glfw_mouse_button);
  glfwSetCursorPosCallback(window, glfw_cursor_pos);
  glfwSetScrollCallback(window, glfw_scroll);
  glfwSetKeyCallback(window, glfw_keyboard);

  std::fprintf(stderr, "glfw_adapter: viewer ready\n");

  while (!exit_request.load() && !glfwWindowShouldClose(window)) {
    if (!paused) {
      StateWire sw;
      if (ReadConsistent(*shm_state, sw)) {
        d->qpos[0] = sw.position[0];
        d->qpos[1] = sw.position[1];
        d->qpos[2] = sw.position[2];
        d->qpos[3] = sw.orientation[0];
        d->qpos[4] = sw.orientation[1];
        d->qpos[5] = sw.orientation[2];
        d->qpos[6] = sw.orientation[3];
        mj_forward(m, d);
      }
    }
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &con);
    glfwSwapBuffers(window);
    glfwPollEvents();
    std::this_thread::sleep_for(milliseconds(16));
  }

  mjr_freeContext(&con);
  mjv_freeScene(&scene);
  munmap(shm_state, sizeof(StateWire));
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
