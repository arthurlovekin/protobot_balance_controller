#include "protobot_balance_controller/protobot_balance_controller.hpp"

namespace protobot_balance_controller
{

ProtobotBalanceController::ProtobotBalanceController()
: controller_interface::ChainableControllerInterface() {}

controller_interface::InterfaceConfiguration ProtobotBalanceController::command_interface_configuration() const
{

}
}