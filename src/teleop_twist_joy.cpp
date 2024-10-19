/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "rightbot_interfaces/srv/motor_recovery.hpp"
#include "rightbot_interfaces/srv/position_control.hpp"
#include "rightbot_interfaces/srv/conveyor_belt_command.hpp"


#include "teleop_twist_joy/teleop_twist_joy.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

namespace teleop_twist_joy
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct TeleopTwistJoy::Impl
{
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string& which_map);
  void sendConveyorRequest(std::vector<double> conveyor_velocity);

  rclcpp::Time to_rclcpp_time(const std_msgs::msg::Header::_stamp_type& stamp);
  double calculateNewVelocity(double velocity_setpoint, double dt, double last_velocity, double accel_limit, double decel_limit);

  // double calculateNewPosition(double position_setpoint, double dt, double last_position, double velocity_limit);
 
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Client<rightbot_interfaces::srv::ConveyorBeltCommand>::SharedPtr conveyor_belt_client;
  
  // Obtain a reference to the underlying rcl_node_t structure
  rclcpp::Node::SharedPtr node;
  bool require_enable_button;

  std::string cmd_vel_topic;
  std::string joy_topic;

  // Store all buttons
  int64_t prev_enable_button;
  int64_t enable_turbo_button;
  double enable_button;

  double conveyor_right_button;
  double conveyor_left_button;
  double conveyor_front_button;
  double conveyor_back_button;
  double conveyor_all_button;

  std::map<int, std::vector<double>> conveyor_vel_map;
  // std::map<int, std::vector<std::string>> conveyor_name_map;
  std::map<int, bool> conveyor_last_button_map;
  // Store all axes and their scales
  std::map<std::string, int64_t> axis_linear_map;
  std::map<std::string, std::map<std::string, double>> scale_linear_map;

  std::map<std::string, int64_t> axis_angular_map;
  std::map<std::string, std::map<std::string, double>> scale_angular_map;

  // Store all limits
  double linear_acceleration_limit;
  double linear_deceleration_limit;

  double angular_acceleration_limit;
  double angular_deceleration_limit;

  double conveyor_right_vel;
  double conveyor_left_vel;
  double conveyor_front_vel;
  double conveyor_back_vel;
  double conveyor_all_vel;

  double last_conveyor_right_vel;
  double last_conveyor_left_vel;
  double last_conveyor_front_vel;
  double last_conveyor_back_vel;

  double velocity_setpoint;

  // Store last execution times

  rclcpp::Time last_joy_time;

  // Store prev state 
  double last_joy_val_x;
  double last_joy_val_y;
  double last_joy_val_yaw;

  bool sent_disable_msg;

  bool last_conveyor_right_button;
  bool last_conveyor_left_button;
  bool last_conveyor_front_button;
  bool last_conveyor_back_button;
  bool last_conveyor_all_button;



  std::map<std::string, double> last_linear_vel;
  std::map<std::string, double> last_angular_vel;
};

/**
 * Constructs TeleopTwistJoy.
 */
TeleopTwistJoy::TeleopTwistJoy(const rclcpp::NodeOptions& options) : Node("teleop_twist_joy_node", options)
{
  pimpl_ = new Impl;

  pimpl_->node = std::shared_ptr<rclcpp::Node>(this);
  pimpl_->joy_topic = this->declare_parameter("joy_topic", "joy");
  this->get_parameter("joy_topic", pimpl_->joy_topic);
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(this->pimpl_->joy_topic, rclcpp::QoS(10),
    std::bind(&TeleopTwistJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  pimpl_->cmd_vel_topic = this->declare_parameter("cmd_vel_topic", "cmd_vel");

  pimpl_->conveyor_belt_client = this->create_client<rightbot_interfaces::srv::ConveyorBeltCommand>("conveyor_belt_command");

  this->get_parameter("cmd_vel_topic", pimpl_->cmd_vel_topic);
  pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(this->pimpl_->cmd_vel_topic, 10);

  pimpl_->require_enable_button = this->declare_parameter("require_enable_button", true);

  pimpl_->conveyor_right_button = this->declare_parameter("conveyor_right_button", 1);
  pimpl_->conveyor_left_button = this->declare_parameter("conveyor_left_button", 3);
  pimpl_->conveyor_front_button = this->declare_parameter("conveyor_front_button", 4);
  pimpl_->conveyor_back_button = this->declare_parameter("conveyor_back_button", 0);
  pimpl_->conveyor_all_button = this->declare_parameter("conveyor_all_button", 6);

  pimpl_->conveyor_right_vel = this->declare_parameter("conveyor_right_vel", 0.5);
  pimpl_->conveyor_left_vel = this->declare_parameter("conveyor_left_vel", 0.5);
  pimpl_->conveyor_front_vel = this->declare_parameter("conveyor_front_vel", 0.5);
  pimpl_->conveyor_back_vel = this->declare_parameter("conveyor_back_vel", 0.5);

  pimpl_->enable_button = this->declare_parameter("enable_button", 7);
  pimpl_->enable_turbo_button = this->declare_parameter("enable_turbo_button", -1);
  std::map<std::string, int64_t> default_linear_map{
    {"x", 5L},
    {"y", -1L},
    {"z", -1L},
  };
  this->declare_parameters("axis_linear", default_linear_map);
  this->get_parameters("axis_linear", pimpl_->axis_linear_map);

  std::map<std::string, int64_t> default_angular_map{
    {"yaw", 2L},
    {"pitch", -1L},
    {"roll", -1L},
  };
  this->declare_parameters("axis_angular", default_angular_map);
  this->get_parameters("axis_angular", pimpl_->axis_angular_map);

  std::map<std::string, double> default_scale_linear_normal_map{
    {"x", 0.0},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear", default_scale_linear_normal_map);
  this->get_parameters("scale_linear", pimpl_->scale_linear_map["normal"]);

  std::map<std::string, double> default_scale_linear_turbo_map{
    {"x", 0.0},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear_turbo", default_scale_linear_turbo_map);
  this->get_parameters("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);

  std::map<std::string, double> default_scale_angular_normal_map{
    {"yaw", 0.0},
    {"pitch", 0.0},
    {"roll", 0.0},
  };
  this->declare_parameters("scale_angular", default_scale_angular_normal_map);
  this->get_parameters("scale_angular", pimpl_->scale_angular_map["normal"]);

  std::map<std::string, double> default_scale_angular_turbo_map{
    {"yaw", 0.0},
    {"pitch", 0.0},
    {"roll", 0.0},
  };
  this->declare_parameters("scale_angular_turbo", default_scale_angular_turbo_map);
  this->get_parameters("scale_angular_turbo", pimpl_->scale_angular_map["turbo"]);

  pimpl_->linear_acceleration_limit = this->declare_parameter("linear_acceleration_limit", 0.4);
  pimpl_->linear_deceleration_limit = this->declare_parameter("linear_deceleration_limit", 0.4);
  pimpl_->angular_acceleration_limit = this->declare_parameter("angular_acceleration_limit", 0.4);
  pimpl_->angular_deceleration_limit = this->declare_parameter("angular_deceleration_limit", 0.4);

  ROS_INFO_COND_NAMED(pimpl_->require_enable_button, "SherlockTeleopJoy",
    "Teleop enable button: RT");
  
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "SherlockTeleopJoy",
    "Turbo button: LB");

  ROS_INFO_COND_NAMED(pimpl_->conveyor_right_button >= 0, "SherlockTeleopJoy",
    "Conveyor right toggle: B");

  ROS_INFO_COND_NAMED(pimpl_->conveyor_left_button >= 0, "SherlockTeleopJoy",
    "Conveyor left toggle: X");

  ROS_INFO_COND_NAMED(pimpl_->conveyor_front_button >= 0, "SherlockTeleopJoy",
    "Conveyor front toggle: Y");

  ROS_INFO_COND_NAMED(pimpl_->conveyor_back_button >= 0, "SherlockTeleopJoy",
    "Conveyor back toggle: A");

  ROS_INFO_COND_NAMED(pimpl_->conveyor_all_button >= 0, "SherlockTeleopJoy",
    "Conveyor all toggle: D-Pad Left");

  ROS_INFO_COND_NAMED(pimpl_->require_enable_button, "SherlockTeleopJoy",
    "X/Y vel control: Left stick");
  
  ROS_INFO_COND_NAMED(pimpl_->require_enable_button, "SherlockTeleopJoy",
    "Yaw vel control: Right stick");

  pimpl_->sent_disable_msg = false;

  auto param_callback =
    [this](std::vector<rclcpp::Parameter> parameters)
    {

      static std::set<std::string> intparams = {"axis_linear.x", "axis_linear.y", "axis_linear.z",
                                                "axis_angular.yaw", "axis_angular.pitch", "axis_angular.roll",
                                                "enable_turbo_button", "conveyor_right_button", "conveyor_left_button", "conveyor_front_button",
                                                "conveyor_back_button"};
      static std::set<std::string> doubleparams = {"scale_linear.x", "scale_linear.y", "scale_linear.z",
                                                  "scale_linear_turbo.x", "scale_linear_turbo.y", "scale_linear_turbo.z", "enable_button",
                                                  "scale_angular.yaw", "scale_angular.pitch", "scale_angular.roll",
                                                  "scale_angular_turbo.yaw", "scale_angular_turbo.pitch", "scale_angular_turbo.roll",
                                                  "linear_acceleration_limit", "linear_deceleration_limit",
                                                  "angular_acceleration_limit", "angular_deceleration_limit", "conveyor_all_button",
                                                  "conveyor_right_vel", "conveyor_left_vel", "conveyor_front_vel", "conveyor_back_vel"};
      
      static std::set<std::string> boolparams = {"require_enable_button"};
      static std::set<std::string> stringparams = {"cmd_vel_topic", "joy_topic"};
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;

      // Loop to check if changed parameters are of expected data type
      for(const auto & parameter : parameters)
      {
        if (intparams.count(parameter.get_name()) == 1)
        {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
          {
            result.reason = "Only integer values can be set for '" + parameter.get_name() + "'.";
            RCLCPP_WARN(this->get_logger(), result.reason.c_str());
            result.successful = false;
            return result;
          }
        }
        else if (doubleparams.count(parameter.get_name()) == 1)
        {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
          {
            result.reason = "Only double values can be set for '" + parameter.get_name() + "'.";
            RCLCPP_WARN(this->get_logger(), result.reason.c_str());
            result.successful = false;
            return result;
          }
        }
        else if (boolparams.count(parameter.get_name()) == 1)
        {
          if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
          {
            result.reason = "Only boolean values can be set for '" + parameter.get_name() + "'.";
            RCLCPP_WARN(this->get_logger(), result.reason.c_str());
            result.successful = false;
            return result;
          }
        }
      }

      // Loop to assign changed parameters to the member variables
      for (const auto & parameter : parameters)
      {
        if (parameter.get_name() == "require_enable_button")
        {
          this->pimpl_->require_enable_button = parameter.get_value<rclcpp::PARAMETER_BOOL>();
        }
        if (parameter.get_name() == "conveyor_right_button")
        {
          this->pimpl_->conveyor_right_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        if (parameter.get_name() == "conveyor_left_button")
        {
          this->pimpl_->conveyor_left_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        if (parameter.get_name() == "conveyor_front_button")
        {
          this->pimpl_->conveyor_front_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        if (parameter.get_name() == "conveyor_back_button")
        {
          this->pimpl_->conveyor_back_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        if (parameter.get_name() == "conveyor_all_button")
        {
          this->pimpl_->conveyor_all_button = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        if (parameter.get_name() == "conveyor_front_vel")
        {
          this->pimpl_->conveyor_front_vel = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        if (parameter.get_name() == "conveyor_back_vel")
        {
          this->pimpl_->conveyor_back_vel = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        if (parameter.get_name() == "conveyor_right_vel")
        {
          this->pimpl_->conveyor_right_vel = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        if (parameter.get_name() == "conveyor_left_vel")
        {
          this->pimpl_->conveyor_left_vel = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        if (parameter.get_name() == "enable_button")
        {
          this->pimpl_->enable_button = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "enable_turbo_button")
        {
          this->pimpl_->enable_turbo_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_linear.x")
        {
          this->pimpl_->axis_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_linear.y")
        {
          this->pimpl_->axis_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_linear.z")
        {
          this->pimpl_->axis_linear_map["z"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_angular.yaw")
        {
          this->pimpl_->axis_angular_map["yaw"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_angular.pitch")
        {
          this->pimpl_->axis_angular_map["pitch"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "axis_angular.roll")
        {
          this->pimpl_->axis_angular_map["roll"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.x")
        {
          this->pimpl_->scale_linear_map["turbo"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.y")
        {
          this->pimpl_->scale_linear_map["turbo"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear_turbo.z")
        {
          this->pimpl_->scale_linear_map["turbo"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.x")
        {
          this->pimpl_->scale_linear_map["normal"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.y")
        {
          this->pimpl_->scale_linear_map["normal"]["y"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_linear.z")
        {
          this->pimpl_->scale_linear_map["normal"]["z"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular_turbo.yaw")
        {
          this->pimpl_->scale_angular_map["turbo"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular_turbo.pitch")
        {
          this->pimpl_->scale_angular_map["turbo"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular_turbo.roll")
        {
          this->pimpl_->scale_angular_map["turbo"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.yaw")
        {
          this->pimpl_->scale_angular_map["normal"]["yaw"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.pitch")
        {
          this->pimpl_->scale_angular_map["normal"]["pitch"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "scale_angular.roll")
        {
          this->pimpl_->scale_angular_map["normal"]["roll"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "linear_acceleration_limit")
        {
          this->pimpl_->linear_acceleration_limit = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "linear_deceleration_limit")
        {
          this->pimpl_->linear_deceleration_limit = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "angular_acceleration_limit")
        {
          this->pimpl_->angular_acceleration_limit = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
        else if (parameter.get_name() == "angular_deceleration_limit")
        {
          this->pimpl_->angular_deceleration_limit = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
      }
      this->pimpl_->velocity_setpoint = 0.0;
      // this->pimpl_->conveyor_name_map[this->pimpl_->conveyor_right_button] = {"conveyor_right"};
      // this->pimpl_->conveyor_name_map[this->pimpl_->conveyor_left_button] = {"conveyor_left"};
      // this->pimpl_->conveyor_name_map[this->pimpl_->conveyor_front_button] = {"conveyor_front"};
      // this->pimpl_->conveyor_name_map[this->pimpl_->conveyor_back_button] = {"conveyor_rear"};
      // this->pimpl_->conveyor_name_map[this->pimpl_->conveyor_all_button] = {"conveyor_front", "conveyor_rear", "conveyor_left", "conveyor_right"};
      this->pimpl_->conveyor_vel_map[this->pimpl_->conveyor_right_button] = {0.0, 0.0, 0.0, this->pimpl_->conveyor_right_vel};
      this->pimpl_->conveyor_vel_map[this->pimpl_->conveyor_left_button] = {0.0, 0.0, this->pimpl_->conveyor_left_vel, 0.0};
      this->pimpl_->conveyor_vel_map[this->pimpl_->conveyor_front_button] = {this->pimpl_->conveyor_front_vel, 0.0, 0.0, 0.0};
      this->pimpl_->conveyor_vel_map[this->pimpl_->conveyor_back_button] = {0.0, this->pimpl_->conveyor_back_vel, 0.0, 0.0};
      this->pimpl_->conveyor_vel_map[this->pimpl_->conveyor_all_button] = {this->pimpl_->conveyor_front_vel, this->pimpl_->conveyor_back_vel, this->pimpl_->conveyor_left_vel, this->pimpl_->conveyor_right_vel};
      this->pimpl_->conveyor_last_button_map[this->pimpl_->conveyor_right_button] = false;
      this->pimpl_->conveyor_last_button_map[this->pimpl_->conveyor_left_button] = false;
      this->pimpl_->conveyor_last_button_map[this->pimpl_->conveyor_front_button] = false;
      this->pimpl_->conveyor_last_button_map[this->pimpl_->conveyor_back_button] = false;
      this->pimpl_->conveyor_last_button_map[this->pimpl_->conveyor_all_button] = false;
      return result;
    };
    callback_handle = this->add_on_set_parameters_callback(param_callback);
  ROS_INFO_COND_NAMED(true, "SherlockTeleopJoy", "helloooooo");
  for (const auto& entry : pimpl_->conveyor_vel_map) {
      std::ostringstream oss;
      oss << "Conveyor " << entry.first << ": Velocities = ";

      for (double vel : entry.second) {
          oss << vel << " ";
      }

      // Log the message using ROS_INFO_COND_NAMED
      ROS_INFO_COND_NAMED(true, "SherlockTeleopJoy",
          "%s", oss.str().c_str());
  }
  };


TeleopTwistJoy::~TeleopTwistJoy()
{
  delete pimpl_;
}

double TeleopTwistJoy::Impl::calculateNewVelocity(double setpoint, double dt, double last_velocity, double accel_limit, double decel_limit)
{
  double max_acceleration = accel_limit * dt;
  double max_deceleration = decel_limit * dt;
  double error = setpoint - last_velocity;
  double new_velocity = last_velocity;

  if (error > 0) 
  { // Need to accelerate
    double acceleration = std::min(error, max_acceleration);
    new_velocity += acceleration;
  }
  else if (error < 0) 
  { // Need to decelerate
    double deceleration = std::min(-error, max_deceleration);
    new_velocity -= deceleration;
  }

  return new_velocity;
}

rclcpp::Time TeleopTwistJoy::Impl::to_rclcpp_time(const std_msgs::msg::Header::_stamp_type& stamp)
{
  // Convert a std_msgs::msg::Header::_stamp_type to rclcpp::Time
  return rclcpp::Time(stamp.sec, stamp.nanosec);
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();

  if(!require_enable_button ||
      (static_cast<int>(joy_msg->axes.size()) > enable_button &&
            (joy_msg->axes[enable_button] < 0.0)))
  {
    if(sent_disable_msg)
    {
      velocity_setpoint = 0.0;
      last_linear_vel["x"] = 0.0;
      last_linear_vel["y"] = 0.0;
      last_angular_vel["yaw"] = 0.0;
    }
    const double dt = (to_rclcpp_time(joy_msg->header.stamp) - to_rclcpp_time(last_joy_time)).seconds();
    // Compute the new linear velocities
    double joy_val = joy_msg->axes[axis_linear_map.at("x")];
    velocity_setpoint = joy_val * scale_linear_map[which_map].at("x");
    cmd_vel_msg->linear.x = calculateNewVelocity(velocity_setpoint, dt, last_linear_vel["x"], linear_acceleration_limit, linear_deceleration_limit);
    last_linear_vel["x"] = cmd_vel_msg->linear.x;
    
    joy_val = joy_msg->axes[axis_linear_map.at("y")];
    velocity_setpoint = joy_val * scale_linear_map[which_map].at("y");
    cmd_vel_msg->linear.y = calculateNewVelocity(velocity_setpoint, dt, last_linear_vel["y"], linear_acceleration_limit, linear_deceleration_limit);
    last_linear_vel["y"] = cmd_vel_msg->linear.y;

    // Compute the new angular velocities
    joy_val = joy_msg->axes[axis_angular_map.at("yaw")];
    velocity_setpoint = joy_val * scale_angular_map[which_map].at("yaw");
    cmd_vel_msg->angular.z = calculateNewVelocity(velocity_setpoint, dt, last_angular_vel["yaw"], angular_acceleration_limit, angular_deceleration_limit);
    last_angular_vel["yaw"] = cmd_vel_msg->angular.z;


    cmd_vel_pub->publish(std::move(cmd_vel_msg));
    sent_disable_msg = false;
    last_joy_time = joy_msg->header.stamp;
  }
  else{
    velocity_setpoint = 0.0;
    last_linear_vel["x"] = 0.0;
    last_linear_vel["y"] = 0.0;
    last_angular_vel["yaw"] = 0.0;
    cmd_vel_pub->publish(std::move(cmd_vel_msg));
    sent_disable_msg = true;
    last_joy_time = joy_msg->header.stamp;
  }
}

// void TeleopTwistJoy::Impl::sendConveyorRequest(std::vector<std::string> conveyor_names, std::vector<double> conveyor_velocity)
// {
//   while (!conveyor_belt_client->wait_for_service(std::chrono::seconds(1))) 
//   {
//     if (!rclcpp::ok())
//     {
//       ROS_INFO_NAMED("SherlockTeleopJoy", "Interrupted while waiting for /conveyor_belt_command service. Exiting.");
//       return;
//     }
//     ROS_INFO_NAMED("SherlockTeleopJoy", "Waiting for /conveyor_belt_command service to appear...");
//   }
//   auto request = std::make_shared<rightbot_interfaces::srv::ConveyorBeltCommand::Request>();


//   request->actuator_name = conveyor_names;
//   request->actuator_velocity = conveyor_velocity;

//   ROS_INFO_COND_NAMED(true, "SherlockTeleopJoy",
//       "Sending Conveyor belt command.");
//   auto future = conveyor_belt_client->async_send_request(request);
  
//   ROS_INFO_COND_NAMED(true, "SherlockTeleopJoy",
//       "Sent Conveyor belt command.");
//   auto result = future.get();

//   if (!result->success)
//   {
//     ROS_INFO_NAMED("SherlockTeleopJoy", "Unable to send conveyor command. Error: %s", result->msg.c_str());
//   } else
//   {
//     ROS_INFO_NAMED("SherlockTeleopJoy",
//     "Successfully sent conveyor command.");
//   }
// }

void TeleopTwistJoy::Impl::sendConveyorRequest(std::vector<double> conveyor_velocity)
{
    // Print the conveyor_names and conveyor_velocity input data directly
    ROS_INFO_NAMED("SherlockTeleopJoy", "Received conveyor request with the following arguments:");
    
    for (size_t i = 0; i < conveyor_velocity.size(); ++i) {
        ROS_INFO_NAMED("SherlockTeleopJoy", "  Actuator[%zu]:, velocity=%f", 
                       i, conveyor_velocity[i]);
    }

    // Check if the service is available
    if (!conveyor_belt_client->wait_for_service(std::chrono::seconds(1))) {
        ROS_INFO_NAMED("SherlockTeleopJoy", "Service /conveyor_belt_command not available after waiting for 1 second.");
        return;
    }

    auto request = std::make_shared<rightbot_interfaces::srv::ConveyorBeltCommand::Request>();
    request->actuator_name = {"conveyor_front", "conveyor_rear", "conveyor_left", "conveyor_right"};;
    request->actuator_velocity = conveyor_velocity;

    ROS_INFO_NAMED("SherlockTeleopJoy", "Sending request to service...");
    auto future = conveyor_belt_client->async_send_request(request);

    ROS_INFO_NAMED("SherlockTeleopJoy", "Request sent. Waiting for response...");

    // Wait for the result with a timeout
    auto status = future.wait_for(std::chrono::seconds(5));
    if (status == std::future_status::timeout) {
        ROS_INFO_NAMED("SherlockTeleopJoy", "Service call timed out after 5 seconds.");
        return;
    } else if (status == std::future_status::deferred) {
        ROS_INFO_NAMED("SherlockTeleopJoy", "Service call was deferred.");
        return;
    } else if (status != std::future_status::ready) {
        ROS_INFO_NAMED("SherlockTeleopJoy", "Unexpected future status.");
        return;
    }

    ROS_INFO_NAMED("SherlockTeleopJoy", "Response received. Processing...");

    auto result = future.get();

    // Print the response details
    ROS_INFO_NAMED("SherlockTeleopJoy", "Response details:");
    ROS_INFO_NAMED("SherlockTeleopJoy", "  Success: %s", result->success ? "true" : "false");
    ROS_INFO_NAMED("SherlockTeleopJoy", "  Message: %s", result->msg.c_str());

    if (!result->success) {
        ROS_INFO_NAMED("SherlockTeleopJoy", "Unable to send conveyor command. Error: %s", result->msg.c_str());
    } else {
        ROS_INFO_NAMED("SherlockTeleopJoy", "Successfully sent conveyor command.");
    }
}


void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{ 
  if ((joy_msg->axes[enable_button] < 0.0) && joy_msg->axes[conveyor_all_button] > 0)
  { 
    if (!last_conveyor_all_button){
    sendConveyorRequest(std::vector<double>{0.42, 1.6, -0.27, 0.27});
    last_conveyor_right_vel = 0.27;
    last_conveyor_left_vel = -0.27;
    last_conveyor_back_vel = 1.6;
    last_conveyor_front_vel = 0.42;
    }
    else {
    sendConveyorRequest(std::vector<double>{0.0, 0.0, 0.0, 0.0});
    last_conveyor_right_vel = 0.0;
    last_conveyor_left_vel  = 0.0;
    last_conveyor_back_vel  = 0.0;
    last_conveyor_front_vel = 0.0;
    }
    last_conveyor_all_button = !last_conveyor_all_button;
  }
  if ((joy_msg->axes[enable_button] < 0.0) && joy_msg->buttons[conveyor_right_button] > 0)
  {
    if (!last_conveyor_right_button) {
    last_conveyor_right_vel = 0.27;
    sendConveyorRequest(std::vector<double>{last_conveyor_front_vel, last_conveyor_back_vel, last_conveyor_left_vel, last_conveyor_right_vel});
    }
    else {
    last_conveyor_right_vel = 0.0;
    sendConveyorRequest(std::vector<double>{last_conveyor_front_vel, last_conveyor_back_vel, last_conveyor_left_vel, last_conveyor_right_vel});
    }
    last_conveyor_right_button = !last_conveyor_right_button;
  }
  if ((joy_msg->axes[enable_button] < 0.0) && joy_msg->buttons[conveyor_left_button] > 0)
  {
    if (!last_conveyor_left_button) {
    last_conveyor_left_vel = -0.27;
    sendConveyorRequest(std::vector<double>{last_conveyor_front_vel, last_conveyor_back_vel, last_conveyor_left_vel, last_conveyor_right_vel});
    }
    else{
    last_conveyor_left_vel  = 0.0;
    sendConveyorRequest(std::vector<double>{last_conveyor_front_vel, last_conveyor_back_vel, last_conveyor_left_vel, last_conveyor_right_vel});
    }
    last_conveyor_left_button = !last_conveyor_left_button;
  }
  if ((joy_msg->axes[enable_button] < 0.0) && joy_msg->buttons[conveyor_front_button] > 0)
  {
    if (!last_conveyor_front_button) {
    last_conveyor_front_vel = 0.42;
    sendConveyorRequest(std::vector<double>{last_conveyor_front_vel, last_conveyor_back_vel, last_conveyor_left_vel, last_conveyor_right_vel});
    }
    else {
    last_conveyor_front_vel = 0.0;
    sendConveyorRequest(std::vector<double>{last_conveyor_front_vel, last_conveyor_back_vel, last_conveyor_left_vel, last_conveyor_right_vel});
    }
    last_conveyor_front_button = !last_conveyor_front_button;
  }
  if ((joy_msg->axes[enable_button] < 0.0) && joy_msg->buttons[conveyor_back_button] > 0)
  {
    if (!last_conveyor_back_button) {
    last_conveyor_back_vel = 1.6;
    sendConveyorRequest(std::vector<double>{last_conveyor_front_vel, last_conveyor_back_vel, last_conveyor_left_vel, last_conveyor_right_vel});
    }
    else {
    last_conveyor_back_vel = 0.0;
    sendConveyorRequest(std::vector<double>{last_conveyor_front_vel, last_conveyor_back_vel, last_conveyor_left_vel, last_conveyor_right_vel});
    }
    last_conveyor_back_button = !last_conveyor_back_button;
  }

  if (enable_turbo_button >= 0 &&
      static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button] && (!require_enable_button ||
    (static_cast<int>(joy_msg->axes.size()) > enable_button &&
          (joy_msg->axes[enable_button] < 0.0))))
  {
    // Check if the axes values were high while enabling controller
    if ((joy_msg->axes[axis_linear_map.at("x")] != 0.0 && prev_enable_button == 0) ||
      (joy_msg->axes[axis_linear_map.at("y")] != 0.0 && prev_enable_button == 0) ||
      (joy_msg->axes[axis_angular_map.at("yaw")] != 0.0 && prev_enable_button == 0))
      {
        return;
      }
    prev_enable_button = 1;
    
    sendCmdVelMsg(joy_msg, "turbo");
  }
  else if (!require_enable_button ||
    (static_cast<int>(joy_msg->axes.size()) > enable_button &&
          (joy_msg->axes[enable_button] < 0.0)))
  {
    // Check if the axes values were high while enabling controller
    if ((joy_msg->axes[axis_linear_map.at("x")] != 0.0 && prev_enable_button == 0) ||
      (joy_msg->axes[axis_linear_map.at("y")] != 0.0 && prev_enable_button == 0) ||
      (joy_msg->axes[axis_angular_map.at("yaw")] != 0.0 && prev_enable_button == 0))
      {
        return;
      }
    prev_enable_button = 1;
    sendCmdVelMsg(joy_msg, "normal");
  }
  else
  {
    // When enable button is unpressed, send no-motion commands
    // in order to stop the robot.

    // Initializes with zeros by default.
    last_joy_time = joy_msg->header.stamp;

    velocity_setpoint = 0.0;
    last_linear_vel["x"] = 0.0;
    last_linear_vel["y"] = 0.0;
    last_angular_vel["yaw"] = 0.0;
    auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel_pub->publish(std::move(cmd_vel_msg));
    sent_disable_msg = true;
  }

}

}// namespace teleop_twist_joy

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_twist_joy::TeleopTwistJoy)
