#ifndef PROTOBOT_BALANCE_CONTROLLER_HPP
#define PROTOBOT_BALANCE_CONTROLLER_HPP

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace protobot_balance_controller
{

class ProtobotBalanceController : public controller_interface::ChainableControllerInterface
{
  public:
    ProtobotBalanceController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // Chainable controller replaces update() with the following two functions
    controller_interface::return_type update_reference_from_subscribers(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    controller_interface::return_type update_and_write_commands(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State & previous_state) override;

  protected:
    bool on_set_chained_mode(bool chained_mode) override;

    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    realtime_tools::RealtimeBuffer<geometry_msgs::msg::TwistStamped> cmd_vel_buffer_;

  // Timeout to consider cmd_vel commands old
  rclcpp::Duration cmd_vel_timeout_ = rclcpp::Duration::from_seconds(0.5);
};

}


#endif