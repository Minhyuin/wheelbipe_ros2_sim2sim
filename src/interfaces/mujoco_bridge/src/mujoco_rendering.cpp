// Copyright (c) 2025 Sangtaek Lee
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "mujoco_ros2_control/mujoco_rendering.hpp"

#include <cstdio>

namespace mujoco_ros2_control {

MujocoRendering* MujocoRendering::instance_ = nullptr;

MujocoRendering* MujocoRendering::get_instance() {
  if (instance_ == nullptr) {
    instance_ = new MujocoRendering();
  }

  return instance_;
}

MujocoRendering* MujocoRendering::get_instance_if_exists() { return instance_; }

MujocoRendering::MujocoRendering()
    : mj_model_(nullptr),
      mj_data_(nullptr),
      mjv_pert_{},
      last_viewport_{0, 0, 0, 0},
      button_left_(false),
      button_middle_(false),
      button_right_(false),
      paused_(false),
      reset_requested_(false),
      ui_mouse_captured_(false),
      base_body_id_(-1),
      lastx_(0.0),
      lasty_(0.0) {}

void MujocoRendering::init(mjModel* mujoco_model, mjData* mujoco_data) {
  mj_model_ = mujoco_model;
  mj_data_ = mujoco_data;

  // create window, make OpenGL context current, request v-sync
  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
  window_ = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window_);

  // initialize visualization data structures
  mjv_defaultCamera(&mjv_cam_);
  mjv_defaultOption(&mjv_opt_);
  mjv_defaultScene(&mjv_scn_);
  mjr_defaultContext(&mjr_con_);
  mjv_defaultPerturb(&mjv_pert_);

  mjv_cam_.type = mjCAMERA_FREE;
  mjv_cam_.distance = 8.;
  base_body_id_ = mj_name2id(mj_model_, mjOBJ_BODY, "base_link");

  // create scene and context
  mjv_makeScene(mj_model_, &mjv_scn_, 2000);
  mjr_makeContext(mj_model_, &mjr_con_, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window_, &MujocoRendering::keyboard_callback);
  glfwSetCursorPosCallback(window_, &MujocoRendering::mouse_move_callback);
  glfwSetMouseButtonCallback(window_, &MujocoRendering::mouse_button_callback);
  glfwSetScrollCallback(window_, &MujocoRendering::scroll_callback);

  // This might cause tearing, but having RViz and the renderer both open can
  // wreak havoc on the rendering process.
  glfwSwapInterval(0);
}

bool MujocoRendering::is_close_flag_raised() { return glfwWindowShouldClose(window_); }

bool MujocoRendering::is_paused() const { return paused_; }

bool MujocoRendering::consume_reset_request() {
  if (!reset_requested_) {
    return false;
  }
  reset_requested_ = false;
  return true;
}

void MujocoRendering::update() {
  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
  glfwMakeContextCurrent(window_);
  last_viewport_ = viewport;

  // Reset the buffer
  mjr_setBuffer(mjFB_WINDOW, &mjr_con_);

  // update scene and render
  mjv_updateScene(mj_model_, mj_data_, &mjv_opt_, mjv_pert_.active ? &mjv_pert_ : NULL,
                  &mjv_cam_, mjCAT_ALL, &mjv_scn_);
  mjr_render(viewport, &mjv_scn_, &mjr_con_);
  draw_control_buttons(viewport);
  draw_base_height_overlay(viewport);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window_);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

void MujocoRendering::close() {
  // free visualization storage
  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);
  glfwDestroyWindow(window_);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif
}

void MujocoRendering::draw_control_buttons(const mjrRect& viewport) {
  constexpr int margin = 14;
  constexpr int width = 112;
  constexpr int height = 34;
  constexpr int gap = 8;
  const int bottom = viewport.height - margin - height;

  mjrRect pause_rect = {margin, bottom, width, height};
  mjrRect reset_rect = {margin + width + gap, bottom, width, height};

  mjr_rectangle(pause_rect, 0.08f, 0.10f, 0.12f, 0.78f);
  mjr_label(pause_rect, mjFONT_NORMAL, paused_ ? "Continue" : "Pause", 0.08f, 0.10f, 0.12f, 0.90f,
            1.0f, 1.0f, 1.0f, &mjr_con_);

  mjr_rectangle(reset_rect, 0.08f, 0.10f, 0.12f, 0.78f);
  mjr_label(reset_rect, mjFONT_NORMAL, "Reset", 0.08f, 0.10f, 0.12f, 0.90f, 1.0f, 1.0f, 1.0f,
            &mjr_con_);

  char status[96];
  std::snprintf(status, sizeof(status), "%s | Space: pause/continue | Backspace: reset",
                paused_ ? "Paused" : "Running");
  mjrRect status_rect = {margin, bottom - height - gap, 2 * width + gap, height};
  mjr_label(status_rect, mjFONT_NORMAL, status, 0.02f, 0.03f, 0.04f, 0.64f, 0.90f, 0.95f, 1.0f,
            &mjr_con_);
}

void MujocoRendering::draw_base_height_overlay(const mjrRect& viewport) {
  if (base_body_id_ < 0 || !mj_data_) {
    return;
  }

  const double base_height = mj_data_->xpos[3 * base_body_id_ + 2];
  char value[32];
  std::snprintf(value, sizeof(value), "%.3f m", base_height);

  mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, "base_link z", value, &mjr_con_);
}

bool MujocoRendering::handle_control_button_click(GLFWwindow* window, double xpos, double ypos) {
  int window_width = 0;
  int window_height = 0;
  int framebuffer_width = 0;
  int framebuffer_height = 0;
  glfwGetWindowSize(window, &window_width, &window_height);
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
  if (window_width <= 0 || window_height <= 0 || framebuffer_width <= 0 ||
      framebuffer_height <= 0) {
    return false;
  }

  const double x =
      xpos * static_cast<double>(framebuffer_width) / static_cast<double>(window_width);
  const double y_from_top =
      ypos * static_cast<double>(framebuffer_height) / static_cast<double>(window_height);
  constexpr int margin = 14;
  constexpr int width = 112;
  constexpr int height = 34;
  constexpr int gap = 8;

  auto inside = [&](int left, int top) {
    return x >= left && x <= left + width && y_from_top >= top && y_from_top <= top + height;
  };

  if (inside(margin, margin)) {
    toggle_pause();
    return true;
  }
  if (inside(margin + width + gap, margin)) {
    request_reset();
    return true;
  }

  return false;
}

void MujocoRendering::request_reset() { reset_requested_ = true; }

void MujocoRendering::toggle_pause() { paused_ = !paused_; }

void MujocoRendering::keyboard_callback(GLFWwindow* window, int key, int scancode, int act,
                                        int mods) {
  get_instance()->keyboard_callback_impl(window, key, scancode, act, mods);
}

void MujocoRendering::mouse_button_callback(GLFWwindow* window, int button, int act, int mods) {
  get_instance()->mouse_button_callback_impl(window, button, act, mods);
}

void MujocoRendering::mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
  get_instance()->mouse_move_callback_impl(window, xpos, ypos);
}

void MujocoRendering::scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
  get_instance()->scroll_callback_impl(window, xoffset, yoffset);
}

void MujocoRendering::keyboard_callback_impl(GLFWwindow* /* window */, int key, int /* scancode */,
                                             int act, int /* mods */) {
  if (act != GLFW_PRESS && act != GLFW_REPEAT) {
    return;
  }

  if (key == GLFW_KEY_SPACE) {
    toggle_pause();
    return;
  }

  if (key == GLFW_KEY_BACKSPACE) {
    request_reset();
    return;
  }
}

void MujocoRendering::mouse_button_callback_impl(GLFWwindow* window, int button, int act,
                                                 int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT && act == GLFW_PRESS) {
    double xpos = 0.0;
    double ypos = 0.0;
    glfwGetCursorPos(window, &xpos, &ypos);
    if (handle_control_button_click(window, xpos, ypos)) {
      ui_mouse_captured_ = true;
      button_left_ = false;
      button_middle_ = false;
      button_right_ = false;
      lastx_ = xpos;
      lasty_ = ypos;
      return;
    }
  }

  // Ctrl + right press grabs the body under the cursor and starts translating it;
  // releasing the right button stops the perturbation.
  if (button == GLFW_MOUSE_BUTTON_RIGHT && act == GLFW_PRESS && (mods & GLFW_MOD_CONTROL)) {
    double xpos = 0.0;
    double ypos = 0.0;
    glfwGetCursorPos(window, &xpos, &ypos);
    if (select_body_for_perturbation(window, xpos, ypos)) {
      mjv_initPerturb(mj_model_, mj_data_, &mjv_scn_, &mjv_pert_);
      mjv_pert_.active = mjPERT_TRANSLATE;
    }
  }
  if (act == GLFW_RELEASE) {
    ui_mouse_captured_ = false;
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
      mjv_pert_.active = 0;
    }
  }

  // update button state
  button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx_, &lasty_);
}

void MujocoRendering::mouse_move_callback_impl(GLFWwindow* window, double xpos, double ypos) {
  if (ui_mouse_captured_) {
    return;
  }

  // no buttons down: nothing to do
  if (!button_left_ && !button_middle_ && !button_right_) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx_;
  double dy = ypos - lasty_;
  lastx_ = xpos;
  lasty_ = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right_) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left_) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move perturb or camera
  if (mjv_pert_.active) {
    // MuJoCo reference GUIs work in bottom-up window coordinates and flip the
    // vertical cursor delta; GLFW reports it top-down, so `dy` is passed through
    // unchanged, exactly like the camera branch below.
    mjv_movePerturb(mj_model_, mj_data_, action, dx / height, dy / height, &mjv_scn_, &mjv_pert_);
  } else {
    mjv_moveCamera(mj_model_, action, dx / height, dy / height, &mjv_scn_, &mjv_cam_);
  }
}

void MujocoRendering::scroll_callback_impl(GLFWwindow* /* window */, double /* xoffset */,
                                           double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(mj_model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &mjv_scn_, &mjv_cam_);
}

bool MujocoRendering::select_body_for_perturbation(GLFWwindow* window, double xpos, double ypos) {
  int window_width = 0;
  int window_height = 0;
  int framebuffer_width = 0;
  int framebuffer_height = 0;
  glfwGetWindowSize(window, &window_width, &window_height);
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
  if (window_width <= 0 || window_height <= 0 || framebuffer_width <= 0 ||
      framebuffer_height <= 0 || last_viewport_.width <= 0 || last_viewport_.height <= 0) {
    return false;
  }

  const double x = xpos * static_cast<double>(framebuffer_width) / window_width;
  const double y_from_top = ypos * static_cast<double>(framebuffer_height) / window_height;
  const double relx = (x - last_viewport_.left) / last_viewport_.width;
  // mjv_select measures the vertical coordinate from the bottom of the viewport.
  const double rely = 1.0 - (y_from_top - last_viewport_.bottom) / last_viewport_.height;
  if (relx < 0.0 || relx > 1.0 || rely < 0.0 || rely > 1.0) {
    return false;
  }

  const mjtNum aspect =
      static_cast<mjtNum>(last_viewport_.width) / static_cast<mjtNum>(last_viewport_.height);
  mjtNum select_point[3] = {0.0, 0.0, 0.0};
  int geom_id[1] = {-1};
  int flex_id[1] = {-1};
  int skin_id[1] = {-1};
  const int body_id = mjv_select(mj_model_, mj_data_, &mjv_opt_, aspect, relx, rely, &mjv_scn_,
                                 select_point, geom_id, flex_id, skin_id);
  if (body_id <= 0) {
    // Non-positive ids mean "nothing selected" or MuJoCo's world body.
    mjv_pert_.select = 0;
    return false;
  }

  mjv_pert_.select = body_id;
  mjv_pert_.flexselect = flex_id[0];
  mjv_pert_.skinselect = skin_id[0];

  mjtNum offset[3];
  mju_sub3(offset, select_point, mj_data_->xpos + 3 * body_id);
  mju_mulMatTVec(mjv_pert_.localpos, mj_data_->xmat + 9 * body_id, offset, 3, 3);
  return true;
}

void MujocoRendering::apply_perturbation() {
  if (!mj_model_ || !mj_data_) {
    return;
  }
  mjv_applyPerturbPose(mj_model_, mj_data_, &mjv_pert_, 0);  // mocap bodies only
  mjv_applyPerturbForce(mj_model_, mj_data_, &mjv_pert_);
}

}  // namespace mujoco_ros2_control
