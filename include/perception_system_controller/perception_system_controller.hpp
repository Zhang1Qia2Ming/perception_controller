#ifndef PERCEPTION_CONTROLLER__PERCEPTION_SYSTEM_CONTROLLER_HPP_
#define PERCEPTION_CONTROLLER__PERCEPTION_SYSTEM_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/msg/image.hpp>

namespace perception_system_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PerceptionSystemController : public controller_interface::ControllerInterface {
    private:
        
    public:
        PerceptionSystemController();

        CallbackReturn on_init() override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;
        
    protected:
        std::shared_ptr<perception_system_controller::ParamListener> param_listener_;
        perception_system_controller::Params params_;
};

} // namespace perception_system_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    perception_system_controller::PerceptionSystemController, controller_interface::ControllerInterface)

#endif // PERCEPTION_CONTROLLER__PERCEPTION_SYSTEM_CONTROLLER_HPP_
