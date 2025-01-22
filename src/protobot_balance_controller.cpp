#include "protobot_balance_controller/protobot_balance_controller.hpp"

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

  // cmd_vel_timeout_seconds_ = rclcpp::Duration::from_seconds(params_.cmd_vel_timeout_seconds);

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
          "Ignoring the received message (timestamp %.10f) because it is older than "
          "the current time by %.10f seconds, which exceeds the allowed timeout (%.4f)",
          msg_time, time_difference,
          params_.cmd_vel_timeout_seconds);
      }
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ProtobotBalanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  std::vector<std::string> configuration_names;
  for (const auto & interface_name : params_.command_interfaces)
  {
    configuration_names.push_back(interface_name);
  }
  command_interfaces_config.names = configuration_names;

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
    RCLCPP_WARN(get_node()->get_logger(), "Velocity message received was a nullptr.");
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

  return controller_interface::return_type::OK;
}

controller_interface::return_type ProtobotBalanceController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // command may be limited further by SpeedLimit,
  // without affecting the stored twist command
  const double linear_command = reference_interfaces_[0];
  const double angular_command = reference_interfaces_[1];
  const double pitch_reference = reference_interfaces_[2];

  if (!std::isfinite(linear_command) || !std::isfinite(angular_command) || !std::isfinite(pitch_reference))
  {
    // NaNs occur on initialization when the reference interfaces are not yet set
    return controller_interface::return_type::OK;
  }

  // If the pitch is lower than the pitch_threshold_degrees_, then pass the command through untouched
  if (pitch_reference < params_.pitch_threshold_degrees)
  {
    // TODO: set 
  }
  else 
  {
    // If the pitch is higher than the pitch_threshold_degrees_, then apply the PID controller
    // TODO: Implement PID controller
    reference_interfaces_[0] = reference_interfaces_[0];
    reference_interfaces_[1] = reference_interfaces_[1];
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
  pitch_setpoint_degrees_ = params_.pitch_0_degrees;
  pitch_error_ = 0.0;
  prev_pitch_error_ = 0.0;
  pitch_integral_ = 0.0;
  return true;
}

std::vector<hardware_interface::CommandInterface> ProtobotBalanceController::on_export_reference_interfaces()
{
  // inputs to this controller: linear command, angular command, and pitch reference
  const int n_reference_interfaces = 3; 
  reference_interfaces_.resize(n_reference_interfaces, std::numeric_limits<double>::quiet_NaN());
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(n_reference_interfaces);

  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("linear_velocity_x"),
    &reference_interfaces_[0]));
  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("angular_velocity_z"),
    &reference_interfaces_[1]));
  reference_interfaces.push_back(hardware_interface::CommandInterface(
    get_node()->get_name(), std::string("pitch_reference"),
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
