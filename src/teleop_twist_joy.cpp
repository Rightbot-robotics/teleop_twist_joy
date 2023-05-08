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
  void resetErrors(std::string motor_name, std::string function_name);
  void sendCmdPosMsg(const sensor_msgs::msg::Joy::SharedPtr);
  void sendJointPoseMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string& which_map, std::string joint_name, bool gripper);
  double capValue(double value, int64_t position);

  rclcpp::Time to_rclcpp_time(const std_msgs::msg::Header::_stamp_type& stamp);
  double calculateNewVelocity(double velocity_setpoint, double dt, double last_velocity, double accel_limit, double decel_limit);

  double calculateNewPosition(double position_setpoint, double dt, double last_position, double velocity_limit);
 
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr cmd_pos_pub;
  rclcpp::Client<rightbot_interfaces::srv::MotorRecovery>::SharedPtr error_reset_client;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pose_pub;
  
  // Obtain a reference to the underlying rcl_node_t structure
  rclcpp::Node::SharedPtr node;
  bool require_enable_button;

  std::string cmd_vel_topic;

  // Store all buttons
  int64_t error_reset_button;
  int64_t enable_button;
  int64_t enable_turbo_button;
  int64_t enable_mode_button;
  int64_t enable_gripper_button;
  int64_t prev_joint_button;
  int64_t next_joint_button;

  // Store current state
  bool mode;
  bool gripper;
  bool sent_disable_msg;
  int64_t joint_index;

  // Store all axes and their scales
  std::map<std::string, int64_t> axis_linear_map;
  std::map<std::string, std::map<std::string, double>> scale_linear_map;

  std::map<std::string, int64_t> axis_angular_map;
  std::map<std::string, std::map<std::string, double>> scale_angular_map;

  std::map<std::string, int64_t> axis_joint_map;
  std::map<std::string, std::map<std::string, double>> scale_joint_map;

  // store pos_linear map and their scale
  
  std::map<std::string, int64_t> pos_linear_map;
  double scale_pos;

  // Store joint names and their limits
  std::vector<std::string> joint_names;
  std::vector<double> joint_limits_min;
  std::vector<double> joint_limits_max;

  // Store all limits
  double linear_acceleration_limit;
  double linear_deceleration_limit;

  double angular_acceleration_limit;
  double angular_deceleration_limit;

  double joint_velocity_limit;
  double velocity_setpoint;

  // Store last execution times
  rclcpp::Time last_error_reset_execution_time;
  rclcpp::Time last_mode_execution_time;
  rclcpp::Time last_pos_execution_time;
  rclcpp::Time last_gripper_execution_time;
  rclcpp::Time last_prev_joint_execution_time;
  rclcpp::Time last_next_joint_execution_time;
  rclcpp::Time last_joy_time;

  // Store prev state 
  double last_joy_val_x;
  double last_joy_val_y;
  double last_joy_val_yaw;
  double last_joy_val_joint;

  std::map<std::string, double> last_linear_vel;
  std::map<std::string, double> last_angular_vel;
  std::map<std::string, double> last_joint_pose;
};

/**
 * Constructs TeleopTwistJoy.
 */
TeleopTwistJoy::TeleopTwistJoy(const rclcpp::NodeOptions& options) : Node("teleop_twist_joy_node", options)
{
  pimpl_ = new Impl;

  pimpl_->node = std::shared_ptr<rclcpp::Node>(this);
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),
    std::bind(&TeleopTwistJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  pimpl_->cmd_pos_pub = this->create_publisher<geometry_msgs::msg::Point>("cmd_pos", 10);
  pimpl_->joint_pose_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states_joy", 10);
  pimpl_->cmd_vel_topic = this->declare_parameter("cmd_vel_topic", "cmd_vel");

  pimpl_->error_reset_client = this->create_client<rightbot_interfaces::srv::MotorRecovery>("motor_recovery");

  this->get_parameter("cmd_vel_topic", pimpl_->cmd_vel_topic);
  pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(this->pimpl_->cmd_vel_topic, 10);

  pimpl_->require_enable_button = this->declare_parameter("require_enable_button", true);

  pimpl_->error_reset_button = this->declare_parameter("error_reset_button", 15);
  pimpl_->enable_button = this->declare_parameter("enable_button", 5);
  pimpl_->enable_turbo_button = this->declare_parameter("enable_turbo_button", -1);
  pimpl_->enable_mode_button = this->declare_parameter("enable_mode_button", 12);
  pimpl_->enable_gripper_button = this->declare_parameter("enable_gripper_button", 0);
  pimpl_->prev_joint_button = this->declare_parameter("prev_joint_button", 3);
  pimpl_->next_joint_button = this->declare_parameter("next_joint_button", 1);


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

  std::map<std::string, int64_t> default_joint_map{
    {"x", 5L},
    {"y", -1L},
    {"z", -1L},
  };
  this->declare_parameters("axis_joint", default_joint_map);
  this->get_parameters("axis_joint", pimpl_->axis_joint_map);

  std::map<std::string, double> default_scale_linear_normal_map{
    {"x", 0.0},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear", default_scale_linear_normal_map);
  this->get_parameters("scale_linear", pimpl_->scale_linear_map["normal"]);

  std::map<std::string, double> default_scale_joint_normal_map{
    {"x", 0.0},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_joint", default_scale_joint_normal_map);
  this->get_parameters("scale_joint", pimpl_->scale_joint_map["normal"]);

  std::map<std::string, double> default_scale_linear_turbo_map{
    {"x", 0.0},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_linear_turbo", default_scale_linear_turbo_map);
  this->get_parameters("scale_linear_turbo", pimpl_->scale_linear_map["turbo"]);

  std::map<std::string, double> default_scale_joint_turbo_map{
    {"x", 0.0},
    {"y", 0.0},
    {"z", 0.0},
  };
  this->declare_parameters("scale_joint_turbo", default_scale_joint_turbo_map);
  this->get_parameters("scale_joint_turbo", pimpl_->scale_joint_map["turbo"]);

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

  std::map<std::string, int64_t> default_pos_linear_map{
    {"x", 0},
    {"y", 0},
    {"z", 0},
  };
  this->declare_parameters("pos_linear", default_pos_linear_map);
  this->get_parameters("pos_linear", pimpl_->pos_linear_map);

  pimpl_->scale_pos = this->declare_parameter("scale_pos", 1.0);

  std::vector<std::string> default_joint_names{
      "base_rotation_joint"   ,
      "camera_rotation_joint" ,
      "elbow_rotation_joint"  , 
      "h_gantry_joint"        ,     
      "rotation1_joint"       ,     
      "rotation2_joint"       ,      
      "v_gantry_joint"        ,       
      "wrist_rotation_joint" };
  pimpl_->joint_names = this->declare_parameter("joint_names", default_joint_names);

  std::vector<double> default_joint_limits_min{
      -3.14,
      -3.14,
      -2.61,
        0.0,
      -1.57,
      -3.14,
        0.0,
      -3.14 };
  pimpl_->joint_limits_min = this->declare_parameter("joint_limits_min", default_joint_limits_min);

  std::vector<double> default_joint_limits_max{
      3.14 ,
      3.14 ,
      2.61 ,
      0.75 ,
      1.57 ,
      3.14 ,
      1.061,
      3.14 };
  pimpl_->joint_limits_max = this->declare_parameter("joint_limits_max", default_joint_limits_max);

  pimpl_->linear_acceleration_limit = this->declare_parameter("linear_acceleration_limit", 0.4);
  pimpl_->linear_deceleration_limit = this->declare_parameter("linear_deceleration_limit", 0.4);
  pimpl_->angular_acceleration_limit = this->declare_parameter("angular_acceleration_limit", 0.4);
  pimpl_->angular_deceleration_limit = this->declare_parameter("angular_deceleration_limit", 0.4);
  pimpl_->joint_velocity_limit = this->declare_parameter("joint_velocity_limit", 0.4);

  ROS_INFO_COND_NAMED(pimpl_->require_enable_button, "SherlockTeleopJoy",
    "Teleop enable button: RB");
  
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "SherlockTeleopJoy",
    "Turbo button: LB");

  ROS_INFO_COND_NAMED(pimpl_->enable_mode_button >= 0, "SherlockTeleopJoy",
    "Base/Manipulator mode switch button: XBOX");
  
  ROS_INFO_COND_NAMED(pimpl_->enable_mode_button >= 0, "SherlockTeleopJoy",
    "Error reset button: B");
  
  ROS_INFO_COND_NAMED(pimpl_->enable_mode_button >= 0, "SherlockTeleopJoy",
    "Pos control: D-Pad");
  
  ROS_INFO_COND_NAMED(pimpl_->enable_mode_button >= 0, "SherlockTeleopJoy",
    "X/Y vel control: Left stick");
  
  ROS_INFO_COND_NAMED(pimpl_->enable_mode_button >= 0, "SherlockTeleopJoy",
    "Yaw vel control: Right stick");

  pimpl_->sent_disable_msg = false;

  auto param_callback =
  [this](std::vector<rclcpp::Parameter> parameters)
  {
    static std::set<std::string> intparams = {"axis_linear.x", "axis_linear.y", "axis_linear.z", "error_reset_button",
                                              "axis_angular.yaw", "axis_angular.pitch", "axis_angular.roll",
                                              "enable_button", "enable_turbo_button", "enable_mode_button", "enable_gripper_button",
                                              "prev_joint_button", "next_joint_button", "axis_joint.x", "pos_linear.x","pos_linear.y"};
    static std::set<std::string> doubleparams = {"scale_linear.x", "scale_linear.y", "scale_linear.z",
                                                 "scale_linear_turbo.x", "scale_linear_turbo.y", "scale_linear_turbo.z",
                                                 "scale_angular.yaw", "scale_angular.pitch", "scale_angular.roll",
                                                 "scale_angular_turbo.yaw", "scale_angular_turbo.pitch", "scale_angular_turbo.roll",
                                                 "linear_acceleration_limit", "linear_deceleration_limit",
                                                 "angular_acceleration_limit", "angular_deceleration_limit",
                                                 "joint_velocity_limit", "scale_joint.x", "scale_joint_turbo.x", "scale_pos"};
    static std::set<std::string> boolparams = {"require_enable_button"};
    static std::set<std::string> stringparams = {"cmd_vel_topic"};
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
      if (parameter.get_name() == "error_reset_button")
      {
        this->pimpl_->error_reset_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      if (parameter.get_name() == "enable_button")
      {
        this->pimpl_->enable_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "enable_turbo_button")
      {
        this->pimpl_->enable_turbo_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "enable_mode_button")
      {
        this->pimpl_->enable_mode_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "enable_gripper_button")
      {
        this->pimpl_->enable_gripper_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "prev_joint_button")
      {
        this->pimpl_->prev_joint_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "next_joint_button")
      {
        this->pimpl_->next_joint_button = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_linear.x")
      {
        this->pimpl_->axis_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "axis_linear.y")
      {
        this->pimpl_->axis_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "pos_linear.x")
      {
        this->pimpl_->pos_linear_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "pos_linear.y")
      {
        this->pimpl_->pos_linear_map["y"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "scale_pos")
      {
        this->pimpl_->scale_pos = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
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
      else if (parameter.get_name() == "axis_ljoint.x")
      {
        this->pimpl_->axis_joint_map["x"] = parameter.get_value<rclcpp::PARAMETER_INTEGER>();
      }
      else if (parameter.get_name() == "scale_joint.x")
      {
        this->pimpl_->scale_joint_map["normal"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "scale_joint_turbo.x")
      {
        this->pimpl_->scale_joint_map["turbo"]["x"] = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
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
      else if (parameter.get_name() == "joint_velocity_limit")
      {
        this->pimpl_->joint_velocity_limit = parameter.get_value<rclcpp::PARAMETER_DOUBLE>();
      }
      else if (parameter.get_name() == "joint_names")
      {
        this->pimpl_->joint_names = parameter.get_value<rclcpp::PARAMETER_STRING_ARRAY>();
      }
      else if (parameter.get_name() == "joint_limits_min")
      {
        this->pimpl_->joint_limits_min = parameter.get_value<rclcpp::PARAMETER_DOUBLE_ARRAY>();
      }
      else if (parameter.get_name() == "joint_limits_max")
      {
        this->pimpl_->joint_limits_max = parameter.get_value<rclcpp::PARAMETER_DOUBLE_ARRAY>();
      }
    }
    for (const std::string & joint_name : this->pimpl_->joint_names)
    {
    this->pimpl_->last_joint_pose[joint_name] = 0.0;
    }
    this->pimpl_->velocity_setpoint = 0.0;
    // create a publisher to publish cmd_vel to the topic name given in cmd_vel_topic param
    return result;
  };

  callback_handle = this->add_on_set_parameters_callback(param_callback);
}

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

double TeleopTwistJoy::Impl::calculateNewPosition(double position_setpoint, double dt, double last_position, double velocity_limit)
{
  double max_velocity = velocity_limit * dt;
  double error = position_setpoint - last_position;
  double new_position = last_position;

  if (error > 0)
  {
    double velocity = std::min(error, max_velocity);
    new_position += velocity;
  }
  else if (error < 0)
  {
    double velocity = std::min(-error, max_velocity);
    new_position -= velocity;
  }

  return new_position;
}


rclcpp::Time TeleopTwistJoy::Impl::to_rclcpp_time(const std_msgs::msg::Header::_stamp_type& stamp)
{
  // Convert a std_msgs::msg::Header::_stamp_type to rclcpp::Time
  return rclcpp::Time(stamp.sec, stamp.nanosec);
}

void TeleopTwistJoy::Impl::sendCmdPosMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  // Initializes with zeros by default.
  auto cmd_pos_msg = std::make_unique<geometry_msgs::msg::Point>();

  if(!require_enable_button ||
      (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
            joy_msg->buttons[enable_button]))
  {
    ROS_INFO_COND_NAMED(enable_mode_button > 0, "SherlockTeleopJoy",
    "sending cmd_pos");
    cmd_pos_msg->x = joy_msg->axes[pos_linear_map.at("x")] * scale_pos;
    cmd_pos_msg->y = joy_msg->axes[pos_linear_map.at("y")] * scale_pos;
    cmd_pos_msg->z = 0.0;
    cmd_pos_pub->publish(std::move(cmd_pos_msg));
  }
}

void TeleopTwistJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();

  if(!require_enable_button ||
      (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
            joy_msg->buttons[enable_button]))
  {
    if(sent_disable_msg)
    {
    velocity_setpoint = 0.0;
    last_linear_vel["x"] = 0.0;
    last_linear_vel["y"] = 0.0;
    last_angular_vel["yaw"] = 0.0;
    }
    // ROS_INFO_COND_NAMED(enable_mode_button > 0, "SherlockTeleopJoy",
    //     "sending cmd_vel");
    const double dt = (to_rclcpp_time(joy_msg->header.stamp) - to_rclcpp_time(last_joy_time)).seconds();
    // // Compute the new linear velocities
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

double TeleopTwistJoy::Impl::capValue(double value, int64_t position)
{
  if(value > joint_limits_max[position])
  {
    return joint_limits_max[position];
  }
  else if (value < joint_limits_min[position])
  {
    return joint_limits_min[position];
  }
  return value;
}

void TeleopTwistJoy::Impl::sendJointPoseMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                         const std::string& which_map, std::string joint_name, bool gripper)
{
  // x axis to increase or decrease position slowly using the same logic.
  auto joint_pose_msg = std::make_unique<sensor_msgs::msg::JointState>();
  joint_pose_msg->header.stamp = joy_msg->header.stamp;

  const double dt = (to_rclcpp_time(joy_msg->header.stamp) - to_rclcpp_time(last_joy_time)).seconds();

  double joy_val = joy_msg->axes[axis_joint_map.at("x")];

  ptrdiff_t pos1 = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), joint_name));

  double position_setpoint =  last_joint_pose[joint_name] + joy_val * (std::abs(joint_limits_min[pos1]) + std::abs(joint_limits_max[pos1])) * scale_joint_map[which_map].at("x");;   

  double new_pose = capValue(calculateNewPosition(position_setpoint, dt, last_joint_pose[joint_name], joint_velocity_limit), pos1);

  joint_pose_msg->name = joint_names;
  // joint_pose_msg->name.push_back("gripper");
  joint_pose_msg->position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::map<std::string, double>::iterator it;
  for(it = last_joint_pose.begin(); it != last_joint_pose.end(); it++)
  { 
    ptrdiff_t pos = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), it->first));
    joint_pose_msg->position[pos] = it->second;
  }
  double gripper_val = gripper? 1.0:0.0;
  // joint_pose_msg->position.push_back(gripper_val);
  joint_pose_pub->publish(std::move(joint_pose_msg));

  last_joint_pose[joint_name] = new_pose;
  last_joy_time = joy_msg->header.stamp;
}

void TeleopTwistJoy::Impl::resetErrors(std::string motor_name, std::string function_name)
{

  while (!error_reset_client->wait_for_service(std::chrono::seconds(1))) 
  {
    if (!rclcpp::ok())
    {
      ROS_INFO_NAMED("SherlockTeleopJoy", "Interrupted while waiting for the service. Exiting.");
      return;
    }
    ROS_INFO_NAMED("SherlockTeleopJoy", "Waiting for the service to appear...");
  }
  ROS_INFO_COND_NAMED(error_reset_button > 0, "SherlockTeleopJoy",
      "Resetting Motor Errors.");
  auto request = std::make_shared<rightbot_interfaces::srv::MotorRecovery::Request>();
  request->motor_name = motor_name;
  request->function_name = function_name;

  auto future = error_reset_client->async_send_request(request);

  // if (future.wait_for(std::chrono::seconds(1)) == std::future_status::timeout)
  // {
  //   error_reset_client->remove_pending_request(future);
  // } else {
  //   auto result = future.get();
  //   // auto result = result_future.get();
    ROS_INFO_NAMED("SherlockTeleopJoy",
      "%s complete for %s", function_name.c_str(), motor_name.c_str());

  // }
  
}

void TeleopTwistJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{ 
  if ((to_rclcpp_time(joy_msg->header.stamp) - to_rclcpp_time(last_mode_execution_time)).seconds() >= 0.5)
  {
    if (enable_mode_button >= 0 && 
      static_cast<int>(joy_msg->buttons.size()) > enable_mode_button &&
      joy_msg->buttons[enable_mode_button])
    {
      mode = !mode;
      if (mode)
      {
        ROS_INFO_COND_NAMED(enable_mode_button > 0, "SherlockTeleopJoy",
        "Controlling Sherlock manipulator now.");

        ROS_INFO_COND_NAMED(enable_gripper_button >= 0, "SherlockTeleopJoy",
        "Gripper toggle button: A");
      }
      else
      {
        ROS_INFO_COND_NAMED(enable_mode_button > 0, "SherlockTeleopJoy",
        "Controlling Sherlock base now.");
      }
        
    }

  last_mode_execution_time = joy_msg->header.stamp;

  }
  
  if(!mode)
  {
    
  
  if (error_reset_button > 0 && 
    static_cast<int>(joy_msg->buttons.size()) > error_reset_button &&
    joy_msg->buttons[error_reset_button])
  {
    if ((to_rclcpp_time(joy_msg->header.stamp) - to_rclcpp_time(last_error_reset_execution_time)).seconds() >= 0.5)
      {
      resetErrors("wheel_1_drive", "RESET_FAULT");
      resetErrors("wheel_2_drive", "RESET_FAULT");
      resetErrors("wheel_1_steer", "RESET_FAULT");
      resetErrors("wheel_2_steer", "RESET_FAULT");
      resetErrors("wheel_1_steer", "RESET_COMMUNICATION");
      resetErrors("wheel_2_steer", "RESET_COMMUNICATION");
      last_error_reset_execution_time = joy_msg->header.stamp;
    }


  }
    if (enable_turbo_button >= 0 &&
        static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button &&
        joy_msg->buttons[enable_turbo_button] && (!require_enable_button ||
      (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
            joy_msg->buttons[enable_button])))
    {
      // check if any pos_linear button is pressed
      if (joy_msg->axes[pos_linear_map.at("x")] != 0.0 ||
          joy_msg->axes[pos_linear_map.at("y")] != 0.0)
        {
          if ((to_rclcpp_time(joy_msg->header.stamp) - to_rclcpp_time(last_pos_execution_time)).seconds() >= 0.5)
          {
            sendCmdPosMsg(joy_msg);
            last_pos_execution_time = joy_msg->header.stamp;
            return;
          }
        }
      else{
        sendCmdVelMsg(joy_msg, "turbo");
      }
    }
    else if (!require_enable_button ||
      (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
            joy_msg->buttons[enable_button]))
    {
      if (joy_msg->axes[pos_linear_map.at("x")] != 0.0 ||
          joy_msg->axes[pos_linear_map.at("y")] != 0.0)
        {
          if ((to_rclcpp_time(joy_msg->header.stamp) - to_rclcpp_time(last_pos_execution_time)).seconds() >= 0.5)
          {
            sendCmdPosMsg(joy_msg);
            last_pos_execution_time = joy_msg->header.stamp;
            return;
          }
        }
      else{
        sendCmdVelMsg(joy_msg, "normal");
      }
    }
    else
    {
      // When enable button is unpressed, send no-motion commands
      // in order to stop the robot.

      // Initializes with zeros by default.
      last_joy_time = joy_msg->header.stamp;
      auto joint_pose_msg = std::make_unique<sensor_msgs::msg::JointState>();
      joint_pose_msg->header.stamp = joy_msg->header.stamp;
      joint_pose_msg->name = joint_names;
      joint_pose_msg->position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      std::map<std::string, double>::iterator it;
      for(it = last_joint_pose.begin(); it != last_joint_pose.end(); it++)
      { 
        ptrdiff_t pos = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), it->first));
        joint_pose_msg->position[pos] = it->second;
      }
      joint_pose_pub->publish(std::move(joint_pose_msg));

      velocity_setpoint = 0.0;
      last_linear_vel["x"] = 0.0;
      last_linear_vel["y"] = 0.0;
      last_angular_vel["yaw"] = 0.0;
      auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_vel_pub->publish(std::move(cmd_vel_msg));
      sent_disable_msg = true;
    }
  }
  else
  {
    // get X/B to choose joint and send apt joint_name
    if ((to_rclcpp_time(joy_msg->header.stamp) - to_rclcpp_time(last_prev_joint_execution_time)).seconds() >= 0.5)
    {
  
      if (prev_joint_button >= 0 && 
        static_cast<int>(joy_msg->buttons.size()) > prev_joint_button &&
        joy_msg->buttons[prev_joint_button])
      { 
        if(joint_index == 0)
        {
          joint_index = static_cast<int>(joint_names.size()) - 1;
        }
        else
        {
          --joint_index;
        }
        const std::string joint = joint_names[joint_index];
        ROS_INFO_COND_NAMED(prev_joint_button > 0, "SherlockTeleopJoy",
        ("Now controlling joint: " + joint).c_str());
      }

      last_prev_joint_execution_time = joy_msg->header.stamp;
    }

    if ((to_rclcpp_time(joy_msg->header.stamp) - to_rclcpp_time(last_next_joint_execution_time)).seconds() >= 0.5)
    {
  
      if (next_joint_button >= 0 && 
        static_cast<int>(joy_msg->buttons.size()) > next_joint_button &&
        joy_msg->buttons[next_joint_button])
      { 
         if(joint_index == static_cast<int>(joint_names.size()) - 1)
        {
          joint_index = 0;
        }
        else
        {
          ++joint_index;
        }
        const std::string joint = joint_names[joint_index];
        ROS_INFO_COND_NAMED(next_joint_button > 0, "SherlockTeleopJoy",
        ("Now controlling joint: " + joint).c_str());
      }

      last_next_joint_execution_time = joy_msg->header.stamp;
    }

    // get A button as toggle switch to send bool gripper command
    if ((to_rclcpp_time(joy_msg->header.stamp) - to_rclcpp_time(last_gripper_execution_time)).seconds() >= 0.5)
    {
  
      if (enable_gripper_button >= 0 && 
          static_cast<int>(joy_msg->buttons.size()) > enable_gripper_button &&
          joy_msg->buttons[enable_gripper_button])
      {
          gripper = !gripper;
          if (gripper)
          {
            ROS_INFO_COND_NAMED(enable_gripper_button > 0, "SherlockTeleopJoy",
            "Gripper ON.");
          }
          else
          {
            ROS_INFO_COND_NAMED(enable_gripper_button > 0, "SherlockTeleopJoy",
            "Gripper OFF.");
          } 
      }

    last_gripper_execution_time = joy_msg->header.stamp;

    }

    if (enable_turbo_button >= 0 &&
        static_cast<int>(joy_msg->buttons.size()) > enable_turbo_button &&
        joy_msg->buttons[enable_turbo_button] && 
        (!require_enable_button || 
          (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
          joy_msg->buttons[enable_button])))
    {
      sendJointPoseMsg(joy_msg, "turbo", joint_names[joint_index], gripper);
    }
    else if (!require_enable_button ||
      (static_cast<int>(joy_msg->buttons.size()) > enable_button &&
            joy_msg->buttons[enable_button]))
    {
      sendJointPoseMsg(joy_msg, "normal", joint_names[joint_index], gripper);
    }
    else
    {
      // When enable button is unpressed, send no-motion commands
      // in order to stop the manipulator.

      // Initializes with zeros by default.
      last_joy_time = joy_msg->header.stamp;
      auto joint_pose_msg = std::make_unique<sensor_msgs::msg::JointState>();

      joint_pose_msg->header.stamp = joy_msg->header.stamp;
      joint_pose_msg->name = joint_names;
      joint_pose_msg->position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      std::map<std::string, double>::iterator it;
      for(it = last_joint_pose.begin(); it != last_joint_pose.end(); it++)
      { 
        ptrdiff_t pos = distance(joint_names.begin(), find(joint_names.begin(), joint_names.end(), it->first));
        joint_pose_msg->position[pos] = it->second;
      }
      joint_pose_pub->publish(std::move(joint_pose_msg));

      velocity_setpoint = 0.0;
      last_linear_vel["x"] = 0.0;
      last_linear_vel["y"] = 0.0;
      last_angular_vel["yaw"] = 0.0;
      auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_vel_pub->publish(std::move(cmd_vel_msg));
      sent_disable_msg = true;
    }
  }
}

}  // namespace teleop_twist_joy

RCLCPP_COMPONENTS_REGISTER_NODE(teleop_twist_joy::TeleopTwistJoy)
