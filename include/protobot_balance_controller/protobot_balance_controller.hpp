#ifndef PROTOBOT_BALANCE_CONTROLLER_HPP
#define PROTOBOT_BALANCE_CONTROLLER_HPP

#include "controller_interface/chainable_controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "realtime_tools/realtime_buffer.hpp"

// auto-generated by generate_parameter_library
#include "protobot_balance_controller_parameters.hpp"

namespace protobot_balance_controller
{
// Note that this whole thing could be implemented using the existing
// PID and passthrough controllers, and calls to the Controller Manager to 
// switch the diff_drive_controller in and out of chained-mode. This would be 
// more reusable, but in this example I'm trying to avoid all the layers of 
// abstraction so that it's easier to understand. 

class ProtobotBalanceController : public controller_interface::ChainableControllerInterface
{
  public:
    // Constructor: empty because work is done in on_init() (as is standard for a pluginlib plugin)
    ProtobotBalanceController();

    // Called once, immediately after the controller plugin is dynamically loaded from the URDF. 
    // Declare node parameters, allocate dynamic memory, and set up communication between the robot and hardware
    controller_interface::CallbackReturn on_init() override;

    // Transition from Unconfigured into inactive state. 
    // Create Publishers and Subscribers, read reconfigurable parameters
    controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & previous_state) override;

    // Define which command interfaces are required by this controller. Could be ALL, NONE, or INDIVIDUAL (in which case, specify which interfaces are required)
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    // Define which state interfaces are required by this controller. 
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    // Transition from inactive to active state. 
    // Reset command interfaces and other internal variables (eg. PID variables) to safe values
    // Make sure this is realtime-safe (minimize allocating dynamic memory and make it fast).
    controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;

    // Transition back from active to inactive state.
    // Make sure this is realtime-safe (minimize allocating dynamic memory and make it fast).
    controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

    // Transition from inactive to unconfigured state.
    controller_interface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State & previous_state) override;

    // Called if the managed node fails a state transition.
    controller_interface::CallbackReturn on_error(
      const rclcpp_lifecycle::State & previous_state) override;

  protected:
    // Parameters and parameter listener auto-generated by generate_parameter_library
    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    //// PID variables ////
    double pitch_setpoint_degrees_;
    double pitch_error_;
    double prev_pitch_error_;
    double pitch_integral_;

    bool reset_pid();

    // For Chainable Controller, we maintain a realtime buffer that can either
    // be filled by the publisher (non-realtime) or when it gets changed by an 
    // upstream controller (realtime). This gets read from in update_and_write_commands()
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::TwistStamped>> cmd_vel_buffer_{nullptr};

    // Chainable controller also has extra functions to switch in and out of chained mode
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    bool on_set_chained_mode(bool chained_mode) override;

    // Chainable controller performs update() in the following two steps: 

    // First it updates its internal references from its subscriber
    // (if in chained-mode this will not be used)
    controller_interface::return_type update_reference_from_subscribers(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    // Then command the control output based on the internal references.
    // This is called repeatedly while the controller is in its active state.
    controller_interface::return_type update_and_write_commands(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscriber_;

};

}


#endif