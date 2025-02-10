#include "protobot_balance_controller/protobot_balance_controller.hpp"
#include "tf2/utils.h"
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
    imu_buffer_ = realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::Imu>>();
    kp_buffer_ = realtime_tools::RealtimeBuffer<double>();
    ki_buffer_ = realtime_tools::RealtimeBuffer<double>();
    kd_buffer_ = realtime_tools::RealtimeBuffer<double>();
    prev_joy_msg_ = std::make_shared<sensor_msgs::msg::Joy>();
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

  // create a tf listener to get the IMU->pendulum_mass transform
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  pendulum_wrt_imu_frame_ = geometry_msgs::msg::TransformStamped();

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

  imu_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<sensor_msgs::msg::Imu> msg) -> void
    {
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
      {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received IMU message with zero timestamp, setting it to current "
          "time, this message will only be shown once");
        msg->header.stamp = get_node()->get_clock()->now();
      }

      const double current_time = get_node()->now().seconds();
      const double msg_time = rclcpp::Time(msg->header.stamp).seconds();
      const double time_difference = current_time - msg_time;
      if (time_difference < params_.cmd_vel_timeout_seconds)
      {
        imu_buffer_.writeFromNonRT(msg);
      }
      else
      {
        RCLCPP_WARN(get_node()->get_logger(), "Ignoring IMU message because it is older than the current time by %.10f seconds, which exceeds the allowed timeout (%.4f)", time_difference, params_.cmd_vel_timeout_seconds);
      }
    });

  joy_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<sensor_msgs::msg::Joy> msg) -> void
    {
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
      {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received joy message with zero timestamp, setting it to current "
          "time, this message will only be shown once");
        msg->header.stamp = get_node()->get_clock()->now();
      }

      const double current_time = get_node()->now().seconds();
      const double msg_time = rclcpp::Time(msg->header.stamp).seconds();
      double time_difference = current_time - msg_time;
      if (time_difference > params_.cmd_vel_timeout_seconds)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Ignoring joy message because it is older than the current time by %.10f seconds, which exceeds the allowed timeout (%.4f)", time_difference, params_.cmd_vel_timeout_seconds);
        return;
      }

      auto linear_map
      {
        [](double x, double x_min, double x_max, double y_min, double y_max) -> double
        {
          return y_min + ((y_max - y_min) / (x_max - x_min)) * (x - x_min);
        }
      };

      // Press A or B button -> enable changing kp or kd{}
      // press right trigger -> raise enabled parameter (kp or kd)
      // press left trigger -> lower enabled parameter (kp or kd)
      if(msg->buttons[0] == 1 || msg->buttons[1] == 1)
      {
        // axes initial value: 1, range [1,-1]. 
        // I want initial value: 0 range [0,1] on right axis and [0,-1] on left
        double right = linear_map(msg->axes[5]-1.0, -2.0, 0.0, 1.0, 0.0);
        double left = linear_map(msg->axes[2]-1.0, -2.0, 0.0, -1.0, 0.0);
        delta_ = right + left;
        
        RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500, "right: %f left: %f, delta= %f", right, left, delta_);
      }

      if(prev_joy_msg_ == nullptr)
      {
        RCLCPP_WARN(get_node()->get_logger(), "previous joy message is null");
        prev_joy_msg_ = msg;
        return;
      }
      const double prev_msg_time = rclcpp::Time(prev_joy_msg_->header.stamp).seconds();
      const double msg_time_difference = msg_time - prev_msg_time;
      if(msg_time_difference > params_.cmd_vel_timeout_seconds)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Ignoring joy message because it is older than the previous message by %.10f seconds, which exceeds the allowed timeout (%.4f)", msg_time_difference, params_.cmd_vel_timeout_seconds);
        prev_joy_msg_ = msg;
        return;
      }
      else if(prev_joy_msg_->buttons.size() <= 0)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Previous Joy message contains no buttons");
        prev_joy_msg_ = msg;
        return;
      }

      bool a_button_pressed = (msg->buttons[0] == 1 && prev_joy_msg_->buttons[0] == 0);
      bool a_button_released = (msg->buttons[0] == 0 && prev_joy_msg_->buttons[0] == 1);
      bool b_button_pressed = (msg->buttons[1] == 1 && prev_joy_msg_->buttons[1] == 0);
      bool b_button_released = (msg->buttons[1] == 0 && prev_joy_msg_->buttons[1] == 1);
      if(a_button_pressed)
      {
        RCLCPP_INFO(get_node()->get_logger(), "A button pressed: editing kp: %f", params_.kp);
      }
      else if(a_button_released)
      {
        params_.kp += delta_;
        RCLCPP_INFO(get_node()->get_logger(), "A button released, kp: %f", params_.kp);

        kp_buffer_.writeFromNonRT(params_.kp);
      }
      if(b_button_pressed)
      {
        RCLCPP_INFO(get_node()->get_logger(), "B button pressed: editing kd: %f", params_.kd);
      }
      else if(b_button_released)
      {
        params_.kd += delta_;
        RCLCPP_INFO(get_node()->get_logger(), "B button released, kd: %f", params_.kd);

        kd_buffer_.writeFromNonRT(params_.kd);
      }

      prev_joy_msg_ = msg;
    });

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
  //// Cmd_vel
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

  //// IMU

  // Get the pendulum frame wrt the IMU frame (if it's not already set)
  if(pendulum_wrt_imu_frame_.header.stamp.sec == 0 && pendulum_wrt_imu_frame_.header.stamp.nanosec == 0)
  {
    try {
      pendulum_wrt_imu_frame_ = tf_buffer_->lookupTransform(
        params_.imu_frame_id,
        params_.pendulum_frame_id, // TODO: is this the right order?
        tf2::TimePointZero
      );
      pendulum_wrt_imu_quat_ = tf2::Quaternion(
        pendulum_wrt_imu_frame_.transform.rotation.x,
        pendulum_wrt_imu_frame_.transform.rotation.y,
        pendulum_wrt_imu_frame_.transform.rotation.z,
        pendulum_wrt_imu_frame_.transform.rotation.w
      );
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "Not updating reference interfaces because could not transform %s to %s (%s). ",
        params_.imu_frame_id.c_str(),
        params_.pendulum_frame_id.c_str(),
        ex.what()
      );
      return controller_interface::return_type::OK;
    }
  }

  // Get the IMU orientation in the world frame
  const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg_ptr = *(imu_buffer_.readFromRT());
  
  if (imu_msg_ptr == nullptr)
  {
    RCLCPP_WARN(get_node()->get_logger(), "IMU message received was a nullptr, which should never happen. Was it configured correctly?");
    return controller_interface::return_type::ERROR;
  }
  else if (!(
    std::isfinite(imu_msg_ptr->orientation.x) &&
    std::isfinite(imu_msg_ptr->orientation.y) &&
    std::isfinite(imu_msg_ptr->orientation.z) &&
    std::isfinite(imu_msg_ptr->orientation.w)))
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      params_.cmd_vel_timeout_seconds * 1000,
      "IMU orientation is not finite. Not updating reference interfaces."
    );
    return controller_interface::return_type::OK;
  }

  tf2::Quaternion imu_wrt_world_quat(
    imu_msg_ptr->orientation.x,
    imu_msg_ptr->orientation.y,
    imu_msg_ptr->orientation.z,
    imu_msg_ptr->orientation.w
  );

  tf2::Quaternion pendulum_wrt_world_quat = imu_wrt_world_quat * pendulum_wrt_imu_quat_;

  // isolate the pitch of the pendulum frame
  double roll_rad, pitch_rad, yaw_rad;
  tf2::Matrix3x3(pendulum_wrt_world_quat).getRPY(roll_rad, pitch_rad, yaw_rad);

  const double pitch_deg = pitch_rad * 180.0 / M_PI;
  if (!std::isfinite(pitch_deg))
  {
    RCLCPP_WARN(get_node()->get_logger(), "Final Calculated Pitch is not finite. Not updating reference interfaces.");
    return controller_interface::return_type::OK;

  }

  reference_interfaces_[2] = pitch_deg;

  //// Joy
  const double kp = *(kp_buffer_.readFromRT());
  const double ki = *(ki_buffer_.readFromRT());
  const double kd = *(kd_buffer_.readFromRT());
  if(!std::isfinite(kp) || !std::isfinite(ki) || !std::isfinite(kd))
  {
    RCLCPP_WARN(get_node()->get_logger(), "KP, KI, or KD is not finite. Not updating reference interfaces.");
    return controller_interface::return_type::OK;
  }
  reference_interfaces_[3] = kp;
  reference_interfaces_[4] = ki;
  reference_interfaces_[5] = kd;

  return controller_interface::return_type::OK;
}

controller_interface::return_type ProtobotBalanceController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double linear_command = reference_interfaces_[0];
  double angular_command = reference_interfaces_[1];
  const double pitch_reference = reference_interfaces_[2];
  const double kp = reference_interfaces_[3];
  const double ki = reference_interfaces_[4];
  const double kd = reference_interfaces_[5];
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500, "KP: %f, KI: %f, KD: %f", kp, ki, kd);

  // Keep wheel command logic independent of pitch_reference, because wheel 
  // commands -> diff_drive_controller -> odom -> ekf -> tf -> pitch_reference

  if (!std::isfinite(linear_command) || !std::isfinite(angular_command))
  {
    // NaNs occur on initialization when the reference interfaces are not yet set
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), params_.cmd_vel_timeout_seconds * 1000,
      "Reference inferfaces are not finite: linear_command= %f, angular_command= %f. Not commanding wheels.", linear_command, angular_command);
    return controller_interface::return_type::OK;
  }

  // If the pitch is lower than the pitch_threshold_degrees_, then pass the command through untouched
  if (!std::isfinite(pitch_reference) || pitch_reference < params_.pitch_threshold_degrees_min || pitch_reference > params_.pitch_threshold_degrees_max)
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

    pitch_setpoint_degrees_ = params_.pitch_setpoint_gain * linear_command;
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
      kp * pitch_error_ +
      ki * pitch_integral_ + 
      kd * pitch_derivative_;
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
  pitch_setpoint_degrees_ = 0.0;
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

  imu_buffer_.reset();
  std::shared_ptr<sensor_msgs::msg::Imu> nan_msg_imu = std::make_shared<sensor_msgs::msg::Imu>();
  nan_msg_imu->header.stamp = get_node()->now();
  nan_msg_imu->orientation.x = std::numeric_limits<double>::quiet_NaN();
  nan_msg_imu->orientation.y = std::numeric_limits<double>::quiet_NaN();
  nan_msg_imu->orientation.z = std::numeric_limits<double>::quiet_NaN();
  nan_msg_imu->orientation.w = std::numeric_limits<double>::quiet_NaN();
  imu_buffer_.writeFromNonRT(nan_msg_imu);

  kp_buffer_.reset();
  ki_buffer_.reset();
  kd_buffer_.reset();
  kp_buffer_.writeFromNonRT(params_.kp);
  ki_buffer_.writeFromNonRT(params_.ki);
  kd_buffer_.writeFromNonRT(params_.kd);

  return true;
}

std::vector<hardware_interface::CommandInterface> ProtobotBalanceController::on_export_reference_interfaces()
{
  // inputs to this controller: linear command, angular command
  const int n_reference_interfaces = 6; 
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
  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("kp"),
    &reference_interfaces_[3]));
  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("ki"),
    &reference_interfaces_[4]));
  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("kd"),
    &reference_interfaces_[5]));
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
