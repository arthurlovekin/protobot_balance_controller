#include "protobot_balance_controller/protobot_balance_controller.hpp"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace protobot_balance_controller
{

ProtobotBalanceController::ProtobotBalanceController()
: controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn ProtobotBalanceController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
    cmd_vel_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::TwistStamped>>();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during ProtobotBalanceController's init stage: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ProtobotBalanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // update parameters if they have changed
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_node()->get_logger(), "Parameters were updated");
  }

  cmd_vel_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) -> void
    {
      // Publisher callback: if the timestamp is valid, add the message to the buffer

      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
      {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received TwistStamped with zero timestamp, setting it to current "
          "time, this message will only be shown once");
        msg->header.stamp = get_node()->get_clock()->now();
      }

      const double current_time = get_node()->now().seconds();
      const double msg_time = rclcpp::Time(msg->header.stamp).seconds();
      const double time_difference = current_time - msg_time;

      if (
        params_.cmd_vel_timeout_seconds == 0.0 ||
        time_difference < params_.cmd_vel_timeout_seconds)
      {
        cmd_vel_buffer_.writeFromNonRT(msg);
      }
      else
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "Ignoring the received TwistStamped message (timestamp %.10f) because"
          " it is older than the current time by %.10f seconds, which exceeds"
          " the allowed timeout (%.4f)",
          msg_time, time_difference,
          params_.cmd_vel_timeout_seconds);
      }
    });

  // create a tf listener to get the pitch of the odom->base_link transform
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ProtobotBalanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names = {
    params_.linear_command_interface,
    params_.angular_command_interface
  };

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration ProtobotBalanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  state_interfaces_config.names = {};
  return state_interfaces_config;
}

controller_interface::CallbackReturn ProtobotBalanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!reset_pid())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ProtobotBalanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ProtobotBalanceController::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  const std::shared_ptr<geometry_msgs::msg::TwistStamped> command_msg_ptr = *(cmd_vel_buffer_.readFromRT());

  if (command_msg_ptr == nullptr)
  {
    RCLCPP_WARN(get_node()->get_logger(), "Velocity message received was a nullptr, which should never happen. Was it configured correctly?");
    return controller_interface::return_type::ERROR;
  }

  const double age_of_last_command = time.seconds() - rclcpp::Time(command_msg_ptr->header.stamp).seconds();

  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > params_.cmd_vel_timeout_seconds)
  {
    reference_interfaces_[0] = 0.0;
    reference_interfaces_[1] = 0.0;
  }
  else if (
    std::isfinite(command_msg_ptr->twist.linear.x) &&
    std::isfinite(command_msg_ptr->twist.angular.z))
  {
    reference_interfaces_[0] = command_msg_ptr->twist.linear.x;
    reference_interfaces_[1] = command_msg_ptr->twist.angular.z;
  }
  else
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), params_.cmd_vel_timeout_seconds * 1000,
      "Command message contains NaNs. Not updating reference interfaces.");
  }

  // Get the pitch of the odom->base_link transform
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      params_.odom_frame_id,
      params_.base_frame_id,
      tf2::TimePointZero
    );
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      params_.cmd_vel_timeout_seconds * 1000,
      "Throttled Error: Not setting pitch reference because could not transform %s to %s (%s)",
      params_.odom_frame_id.c_str(),
      params_.base_frame_id.c_str(),
      ex.what()
    );
    return controller_interface::return_type::OK;
  }

  tf2::Quaternion orientation(
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w
  );
  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  if (std::isfinite(pitch))
  {
    reference_interfaces_[2] = pitch;
  }
  else
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      params_.cmd_vel_timeout_seconds * 1000,
      "Transform pitch is not finite. Not updating reference interfaces."
    );
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type ProtobotBalanceController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double linear_command = reference_interfaces_[0];
  double angular_command = reference_interfaces_[1];
  const double pitch_reference = reference_interfaces_[2];

  if (!std::isfinite(linear_command) || !std::isfinite(angular_command) || !std::isfinite(pitch_reference))
  {
    // NaNs occur on initialization when the reference interfaces are not yet set
    return controller_interface::return_type::OK;
  }

  // If the pitch is lower than the pitch_threshold_degrees_, then pass the command through untouched
  if (pitch_reference < params_.pitch_threshold_degrees)
  {
    if(pid_enabled_)
    {
      pid_enabled_ = false;
      RCLCPP_INFO(get_node()->get_logger(), "PID controller disabled");
    }
  }
  else 
  {
    // If the pitch is higher than the pitch_threshold_degrees_, then apply the PID controller
    // TODO: Implement PID controller
    if (!pid_enabled_) 
    {
      pid_enabled_ = true;
      RCLCPP_INFO(get_node()->get_logger(), "PID controller enabled");
    }
    if (param_listener_->is_old(params_))
    {
      params_ = param_listener_->get_params();
      RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "PID parameters were updated (logs every second)");
    }

    pitch_setpoint_degrees_ = params_.pitch_0_degrees + params_.setpoint_gain * linear_command;
    pitch_error_ = pitch_reference - pitch_setpoint_degrees_;
    pitch_derivative_ = (pitch_error_ - prev_pitch_error_) / period.seconds();
    pitch_integral_ += pitch_error_ * period.seconds();
    if (pitch_integral_ > params_.antiwindup_integral_max)
    {
      pitch_integral_ = params_.antiwindup_integral_max;
    }
    else if (pitch_integral_ < params_.antiwindup_integral_min)
    {
      pitch_integral_ = params_.antiwindup_integral_min;
    }

    linear_command = 
      params_.kp * pitch_error_ +
      params_.ki * pitch_integral_ + 
      params_.kd * pitch_derivative_;
    angular_command = params_.angular_velocity_gain * angular_command;
    prev_pitch_error_ = pitch_error_;
  }

  // Set the command interfaces

  if(!command_interfaces_[0].set_value(linear_command))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set linear command");
    return controller_interface::return_type::ERROR;
  }
  if(!command_interfaces_[1].set_value(angular_command))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set angular command");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ProtobotBalanceController::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  if (!reset_pid())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Cleaning up ProtobotBalanceController from the %s state", previous_state.label().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ProtobotBalanceController::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  if (!reset_pid())
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Handling error from ProtobotBalanceController's %s state", previous_state.label().c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

bool ProtobotBalanceController::reset_pid()
{
  pid_enabled_ = false;
  pitch_setpoint_degrees_ = params_.pitch_0_degrees;
  pitch_error_ = 0.0;
  prev_pitch_error_ = 0.0;
  pitch_integral_ = 0.0;
  pitch_derivative_ = 0.0;

  // Empty the cmd_vel_buffer_ then fill it with a single NaN
  cmd_vel_buffer_.reset();
  std::shared_ptr<geometry_msgs::msg::TwistStamped> nan_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();
  nan_msg->header.stamp = get_node()->now();
  nan_msg->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  nan_msg->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
  cmd_vel_buffer_.writeFromNonRT(nan_msg);
  return true;
}

std::vector<hardware_interface::CommandInterface> ProtobotBalanceController::on_export_reference_interfaces()
{
  // inputs to this controller: linear command, angular command
  const int n_reference_interfaces = 3; 
  reference_interfaces_.resize(n_reference_interfaces, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(n_reference_interfaces);

  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("linear"),
    &reference_interfaces_[0]));
  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("angular"),
    &reference_interfaces_[1]));
  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("pitch"),
    &reference_interfaces_[2]));

  return reference_interfaces;
}

bool ProtobotBalanceController::on_set_chained_mode(bool chained_mode)
{
  (void)chained_mode;
  return true;
}

} // namespace protobot_balance_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(protobot_balance_controller::ProtobotBalanceController, controller_interface::ChainableControllerInterface)
