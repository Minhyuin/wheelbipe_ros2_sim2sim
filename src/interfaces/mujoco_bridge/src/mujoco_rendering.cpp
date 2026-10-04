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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace mujoco_ros2_control {
namespace {

// Control panel geometry in window coordinates with the origin at the top-left,
// which is how GLFW reports the cursor. mjr draws with the origin at the
// bottom-left, so every rectangle goes through to_mjr().
constexpr int kMargin = 14;
constexpr int kButtonWidth = 112;
constexpr int kButtonHeight = 34;
constexpr int kGap = 8;
constexpr int kRowStep = kButtonHeight + kGap;
constexpr int kLabelWidth = 150;
constexpr int kTrackWidth = 260;
constexpr int kTrackHeight = 18;
constexpr int kTrackX = kMargin + kLabelWidth;
constexpr int kStatusWidth = 500;
constexpr int kStopWidth = 72;
constexpr int kStateButtonWidth = 64;
constexpr int kStateButtonCount = 4;
constexpr int kSliderRows = 3;
constexpr int kActionsRow = 4;
constexpr int kStatusRow = 5;

// The controller falls back to its safe defaults once commands stop arriving
// (motion_command_timeout_sec in template_ros2_controller_parameters.yaml).
constexpr double kTeleopTimeoutSec = 0.5;
// Command values are compared against the last value this viewer published to
// tell the echo of our own message apart from a keyboard command.
constexpr double kCommandEpsilon = 1e-9;

const char* const kStateNames[kStateButtonCount] = {"INIT", "IDLE", "PREP", "RL"};

struct UiRect {
  int x;
  int y;  // distance from the top edge
  int w;
  int h;
};

// Convert a top-left anchored rectangle to mjr's bottom-left anchored one.
mjrRect to_mjr(const UiRect& rect, int viewport_height) {
  return {rect.x, viewport_height - rect.y - rect.h, rect.w, rect.h};
}

bool contains(const UiRect& rect, double x, double y) {
  return x >= rect.x && x <= rect.x + rect.w && y >= rect.y && y <= rect.y + rect.h;
}

// Slider labels, ranges and units. The ranges mirror the controller clamps in
// template_ros2_controller_parameters.yaml (command_height_min/max,
// motion_linear_x_min/max, motion_angular_z_min/max).
struct SliderSpec {
  const char* label;
  const char* unit;
  double min;
  double max;
  int decimals;
};

constexpr SliderSpec kSliders[kSliderRows] = {
    {"Leg height", "m", 0.20, 0.40, 3},
    {"Linear vel", "m/s", -2.5, 2.5, 2},
    {"Yaw rate", "rad/s", -3.0, 3.0, 2},
};

UiRect slider_track(int index) {
  return {kTrackX, kMargin + kRowStep * (index + 1) + (kButtonHeight - kTrackHeight) / 2,
          kTrackWidth, kTrackHeight};
}

}  // namespace

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
      lasty_(0.0),
      viewer_teleop_enabled_(true),
      viewer_owns_teleop_(false),
      target_height_(0.22),
      target_linear_x_(0.0),
      target_angular_z_(0.0),
      published_height_(0.22),
      published_linear_x_(0.0),
      published_angular_z_(0.0),
      teleop_mirror_received_(false),
      teleop_mirror_stamp_{},
      live_state_(-1),
      state_request_(-1),
      active_slider_(-1) {}

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
  // The checker floor has reflectance 0.2 and mirrors the robot, which is noisy
  // when inspecting the gait. mjv_makeScene resets the render flags, so this has
  // to come after it. The floor texture, shadows and skybox stay enabled.
  mjv_scn_.flags[mjRND_REFLECTION] = 0;

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
  const int viewport_height = viewport.height;

  const mjrRect pause_rect =
      to_mjr({kMargin, kMargin, kButtonWidth, kButtonHeight}, viewport_height);
  const mjrRect reset_rect = to_mjr({kMargin + kButtonWidth + kGap, kMargin, kButtonWidth,
                                     kButtonHeight}, viewport_height);

  mjr_rectangle(pause_rect, 0.08f, 0.10f, 0.12f, 0.78f);
  mjr_label(pause_rect, mjFONT_NORMAL, paused_ ? "Continue" : "Pause", 0.08f, 0.10f, 0.12f, 0.90f,
            1.0f, 1.0f, 1.0f, &mjr_con_);

  mjr_rectangle(reset_rect, 0.08f, 0.10f, 0.12f, 0.78f);
  mjr_label(reset_rect, mjFONT_NORMAL, "Reset", 0.08f, 0.10f, 0.12f, 0.90f, 1.0f, 1.0f, 1.0f,
            &mjr_con_);

  if (!viewer_teleop_enabled_) {
    return;
  }

  const bool mirror_stale =
      !teleop_mirror_received_ ||
      std::chrono::duration<double>(std::chrono::steady_clock::now() - teleop_mirror_stamp_)
              .count() > kTeleopTimeoutSec;

  for (int index = 0; index < kSliderRows; ++index) {
    const SliderSpec& spec = kSliders[index];
    const UiRect track = slider_track(index);
    const mjrRect track_rect = to_mjr(track, viewport_height);

    // The controller zeroes the velocity commands when they time out, so an idle
    // mirror has to show 0 as well. The height command keeps its last value.
    double display = slider_value(index);
    if (!viewer_owns_teleop_ && mirror_stale && index > 0) {
      display = 0.0;
    }

    const double fraction = std::clamp((display - spec.min) / (spec.max - spec.min), 0.0, 1.0);

    mjr_rectangle(track_rect, 0.10f, 0.12f, 0.15f, 0.85f);
    mjrRect filled = track_rect;
    filled.width = static_cast<int>(track_rect.width * fraction);
    mjr_rectangle(filled, 0.20f, 0.52f, 0.88f, 0.55f);
    const int knob_left = std::clamp(track_rect.left + filled.width - 3, track_rect.left,
                                     track_rect.left + track_rect.width - 6);
    const mjrRect knob = {knob_left, track_rect.bottom - 4, 6, track_rect.height + 8};
    mjr_rectangle(knob, 0.88f, 0.92f, 1.0f, 0.95f);

    char value[32];
    std::snprintf(value, sizeof(value), "%.*f %s", spec.decimals, display, spec.unit);

    const int text_y = track.y - (kButtonHeight - kTrackHeight) / 2;
    const mjrRect label_rect =
        to_mjr({kMargin, text_y, kLabelWidth - 10, kButtonHeight}, viewport_height);
    mjr_label(label_rect, mjFONT_NORMAL, spec.label, 0.02f, 0.03f, 0.04f, 0.55f, 0.86f, 0.89f,
              0.93f, &mjr_con_);
    const mjrRect value_rect =
        to_mjr({kTrackX + kTrackWidth + 10, text_y, 110, kButtonHeight}, viewport_height);
    mjr_label(value_rect, mjFONT_NORMAL, value, 0.02f, 0.03f, 0.04f, 0.55f, 0.95f, 0.97f, 1.0f,
              &mjr_con_);
  }

  const int actions_y = kMargin + kRowStep * kActionsRow;
  const mjrRect stop_rect =
      to_mjr({kMargin, actions_y, kStopWidth, kButtonHeight}, viewport_height);
  mjr_label(stop_rect, mjFONT_NORMAL, "Stop", 0.32f, 0.12f, 0.10f, 0.90f, 1.0f, 1.0f, 1.0f,
            &mjr_con_);

  for (int state = 0; state < kStateButtonCount; ++state) {
    const int left = kMargin + kStopWidth + kGap + state * (kStateButtonWidth + kGap);
    const mjrRect rect =
        to_mjr({left, actions_y, kStateButtonWidth, kButtonHeight}, viewport_height);
    const bool active = live_state_ == state;
    const float background[3] = {active ? 0.16f : 0.08f, active ? 0.42f : 0.10f,
                                 active ? 0.62f : 0.12f};
    mjr_label(rect, mjFONT_NORMAL, kStateNames[state], background[0], background[1], background[2],
              0.88f, 1.0f, 1.0f, 1.0f, &mjr_con_);
  }

  char status[160];
  std::snprintf(status, sizeof(status), "%s | GUI cmd: %s | Space: pause | Backspace: reset",
                paused_ ? "Paused" : "Running", viewer_owns_teleop_ ? "on" : "off");
  const mjrRect status_rect =
      to_mjr({kMargin, kMargin + kRowStep * kStatusRow, kStatusWidth, kButtonHeight},
             viewport_height);
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

  if (contains({kMargin, kMargin, kButtonWidth, kButtonHeight}, x, y_from_top)) {
    toggle_pause();
    return true;
  }
  if (contains({kMargin + kButtonWidth + kGap, kMargin, kButtonWidth, kButtonHeight}, x, y_from_top)) {
    request_reset();
    return true;
  }

  if (!viewer_teleop_enabled_) {
    return false;
  }

  for (int index = 0; index < kSliderRows; ++index) {
    const UiRect track = slider_track(index);
    // Grab the full row height, not just the thin rail.
    if (contains({track.x, track.y - 8, track.w, track.h + 16}, x, y_from_top)) {
      active_slider_ = index;
      set_slider_from_window_x(index, x);
      return true;
    }
  }

  const int actions_y = kMargin + kRowStep * kActionsRow;
  if (contains({kMargin, actions_y, kStopWidth, kButtonHeight}, x, y_from_top)) {
    target_linear_x_ = 0.0;
    target_angular_z_ = 0.0;
    viewer_owns_teleop_ = true;
    return true;
  }

  for (int state = 0; state < kStateButtonCount; ++state) {
    const int left = kMargin + kStopWidth + kGap + state * (kStateButtonWidth + kGap);
    if (contains({left, actions_y, kStateButtonWidth, kButtonHeight}, x, y_from_top)) {
      state_request_ = state;
      return true;
    }
  }

  // The status line doubles as the GUI/keyboard handover toggle.
  if (contains({kMargin, kMargin + kRowStep * kStatusRow, kStatusWidth, kButtonHeight}, x,
               y_from_top)) {
    viewer_owns_teleop_ = !viewer_owns_teleop_;
    return true;
  }

  return false;
}

void MujocoRendering::request_reset() { reset_requested_ = true; }

void MujocoRendering::toggle_pause() { paused_ = !paused_; }

void MujocoRendering::apply_slider_drag(GLFWwindow* window, double xpos) {
  int window_width = 0;
  int window_height = 0;
  int framebuffer_width = 0;
  int framebuffer_height = 0;
  glfwGetWindowSize(window, &window_width, &window_height);
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
  if (window_width <= 0 || framebuffer_width <= 0) {
    return;
  }
  set_slider_from_window_x(active_slider_, xpos * static_cast<double>(framebuffer_width) /
                                              static_cast<double>(window_width));
}

void MujocoRendering::set_slider_from_window_x(int index, double x) {
  if (index < 0 || index >= kSliderRows) {
    return;
  }
  const UiRect track = slider_track(index);
  const SliderSpec& spec = kSliders[index];
  const double fraction = std::clamp((x - track.x) / static_cast<double>(track.w), 0.0, 1.0);
  const double value = spec.min + fraction * (spec.max - spec.min);
  switch (index) {
    case 0:
      target_height_ = value;
      break;
    case 1:
      target_linear_x_ = value;
      break;
    default:
      target_angular_z_ = value;
      break;
  }
  viewer_owns_teleop_ = true;
}

double MujocoRendering::slider_value(int index) const {
  switch (index) {
    case 0:
      return target_height_;
    case 1:
      return target_linear_x_;
    default:
      return target_angular_z_;
  }
}

void MujocoRendering::set_viewer_teleop_enabled(bool enabled) {
  if (viewer_teleop_enabled_ == enabled) {
    return;
  }
  viewer_teleop_enabled_ = enabled;
  if (!enabled) {
    viewer_owns_teleop_ = false;
    active_slider_ = -1;
  }
}

bool MujocoRendering::viewer_teleop_enabled() const { return viewer_teleop_enabled_; }

bool MujocoRendering::viewer_owns_teleop() const { return viewer_owns_teleop_; }

double MujocoRendering::target_height() const { return target_height_; }

double MujocoRendering::target_linear_velocity() const { return target_linear_x_; }

double MujocoRendering::target_angular_velocity() const { return target_angular_z_; }

void MujocoRendering::note_published_teleop_command(double height, double linear_x,
                                                   double angular_z) {
  published_height_ = height;
  published_linear_x_ = linear_x;
  published_angular_z_ = angular_z;
}

void MujocoRendering::mirror_teleop_velocity(double linear_x, double angular_z) {
  teleop_mirror_received_ = true;
  teleop_mirror_stamp_ = std::chrono::steady_clock::now();
  if (std::abs(linear_x - published_linear_x_) <= kCommandEpsilon &&
      std::abs(angular_z - published_angular_z_) <= kCommandEpsilon) {
    return;  // Echo of the value this viewer just published.
  }
  // Somebody else is driving the velocity topics; hand control back to them.
  viewer_owns_teleop_ = false;
  target_linear_x_ = linear_x;
  target_angular_z_ = angular_z;
}

void MujocoRendering::mirror_teleop_height(double height) {
  teleop_mirror_received_ = true;
  teleop_mirror_stamp_ = std::chrono::steady_clock::now();
  if (std::abs(height - published_height_) <= kCommandEpsilon) {
    return;  // Echo of the value this viewer just published.
  }
  viewer_owns_teleop_ = false;
  target_height_ = height;
}

void MujocoRendering::set_live_state(int state) { live_state_ = state; }

bool MujocoRendering::consume_state_request(int& state) {
  if (state_request_ < 0) {
    return false;
  }
  state = state_request_;
  state_request_ = -1;
  return true;
}

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
    active_slider_ = -1;
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
  if (active_slider_ >= 0) {
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) != GLFW_PRESS) {
      // The button was released outside the window, so GLFW never delivered the
      // release event: end the drag here instead of following the cursor.
      active_slider_ = -1;
      ui_mouse_captured_ = false;
      return;
    }
    apply_slider_drag(window, xpos);
    lastx_ = xpos;
    lasty_ = ypos;
    return;
  }

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
