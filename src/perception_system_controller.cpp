#include "perception_system_controller/perception_system_controller.hpp"
#include "perception_hardware/mock_camera_data.hpp"
#include "controller_interface/helpers.hpp"


namespace perception_system_controller {
    PerceptionSystemController::PerceptionSystemController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn PerceptionSystemController::on_init()
    {
        try
        {
            RCLCPP_INFO(get_node()->get_logger(), "PerceptionSystemController on_init");
            param_listener_ = std::make_shared<perception_system_controller::ParamListener>(get_node());
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during controller's init with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PerceptionSystemController::on_configure(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(get_node()->get_logger(), "PerceptionSystemController on_configure");
        params_ = param_listener_->get_params();

        
        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PerceptionSystemController::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PerceptionSystemController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PerceptionSystemController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }


} // namespace perception_system_controller
