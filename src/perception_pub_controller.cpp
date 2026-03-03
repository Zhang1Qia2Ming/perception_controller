#include "perception_pub_controller/perception_pub_controller.hpp"

namespace perception_controller {
    PerceptionPubController::PerceptionPubController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn PerceptionPubController::on_init() {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PerceptionPubController::on_configure(const rclcpp_lifecycle::State& previous_state) {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PerceptionPubController::on_activate(const rclcpp_lifecycle::State& previous_state) {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PerceptionPubController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;
        return config;
    }

    controller_interface::InterfaceConfiguration PerceptionPubController::command_interface_configuration() const {
        return controller_interface::InterfaceConfiguration();
    }

    controller_interface::CallbackReturn PerceptionPubController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        return controller_interface::CallbackReturn::SUCCESS;
    }

} // namespace perception_controller

