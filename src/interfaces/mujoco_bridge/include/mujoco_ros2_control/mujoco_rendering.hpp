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

#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_RENDERING_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_RENDERING_HPP_

#include <chrono>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

namespace mujoco_ros2_control {

class MujocoRendering {
 public:
  MujocoRendering(const MujocoRendering& obj) = delete;
  void operator=(const MujocoRendering&) = delete;

  static MujocoRendering* get_instance();
  /// Viewer instance, or nullptr when the viewer was never created (headless runs).
  static MujocoRendering* get_instance_if_exists();
  void init(mjModel* mujoco_model, mjData* mujoco_data);
  bool is_close_flag_raised();
  bool is_paused() const;
  bool consume_reset_request();
  void update();
  void close();
  /// Write the interactive drag perturbation into mjData->xfrc_applied. Must be
  /// called immediately before stepping; no-op while no body is being dragged.
  void apply_perturbation();

  /// Teleop panel (sliders plus the state row). The panel mirrors the commands
  /// keyboard_teleop publishes on the command topics, and only takes those
  /// topics over once one of its widgets is used.
  void set_viewer_teleop_enabled(bool enabled);
  bool viewer_teleop_enabled() const;
  bool viewer_owns_teleop() const;
  double target_height() const;
  double target_linear_velocity() const;
  double target_angular_velocity() const;
  /// Record the command the ROS bridge just published, so the copy of our own
  /// message coming back through the subscription is not mistaken for keyboard
  /// input.
  void note_published_teleop_command(double height, double linear_x, double angular_z);
  /// Mirror a message published on the command topics by another node. Our own
  /// echo is ignored; any other value takes control back from the viewer.
  void mirror_teleop_velocity(double linear_x, double angular_z);
  void mirror_teleop_height(double height);
  /// State reported by the controller's `current_state`, used to highlight the
  /// matching state button. Pass a negative value when unknown.
  void set_live_state(int state);
  bool consume_state_request(int& state);

 private:
  MujocoRendering();
  static void keyboard_callback(GLFWwindow* window, int key, int scancode, int act, int mods);
  static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);
  static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos);
  static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

  void keyboard_callback_impl(GLFWwindow* window, int key, int scancode, int act, int mods);
  void mouse_button_callback_impl(GLFWwindow* window, int button, int act, int mods);
  void mouse_move_callback_impl(GLFWwindow* window, double xpos, double ypos);
  void scroll_callback_impl(GLFWwindow* window, double xoffset, double yoffset);
  void draw_control_buttons(const mjrRect& viewport);
  void draw_base_height_overlay(const mjrRect& viewport);
  bool handle_control_button_click(GLFWwindow* window, double xpos, double ypos);
  void apply_slider_drag(GLFWwindow* window, double xpos);
  void set_slider_from_window_x(int index, double x);
  double slider_value(int index) const;
  bool select_body_for_perturbation(GLFWwindow* window, double xpos, double ypos);
  void request_reset();
  void toggle_pause();

  static MujocoRendering* instance_;

  mjModel* mj_model_;
  mjData* mj_data_;

  // Window and primary camera for the simulation's viewer
  GLFWwindow* window_;
  mjvCamera mjv_cam_;

  // Options for the rendering context and scene, all of these are hard coded to defaults.
  mjvOption mjv_opt_;
  mjvScene mjv_scn_;
  mjrContext mjr_con_;

  // Interactive drag perturbation, driven by Ctrl + right-drag in the window.
  mjvPerturb mjv_pert_;
  // Framebuffer rectangle of the last rendered frame, used to pick bodies.
  mjrRect last_viewport_;

  bool button_left_;
  bool button_middle_;
  bool button_right_;
  bool paused_;
  bool reset_requested_;
  bool ui_mouse_captured_;
  int base_body_id_;
  double lastx_;
  double lasty_;

  // Teleop panel state, shared with the ROS bridge.
  bool viewer_teleop_enabled_;
  bool viewer_owns_teleop_;
  double target_height_;
  double target_linear_x_;
  double target_angular_z_;
  // Last command this viewer published, used to recognize the echo of our own
  // messages on the command subscriptions.
  double published_height_;
  double published_linear_x_;
  double published_angular_z_;
  bool teleop_mirror_received_;
  std::chrono::steady_clock::time_point teleop_mirror_stamp_;
  int live_state_;
  int state_request_;
  int active_slider_;
};
}  // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_RENDERING_HPP_
