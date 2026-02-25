#include "controller_interface/controller_interface.hpp"
#include "perception_hardware/t265_camera_data.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"


namespace perception_controller {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class TestT265CameraController : public controller_interface::ControllerInterface {

        private:
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

        public:
            CallbackReturn on_init() override {
                RCLCPP_INFO(rclcpp::get_logger("TestT265CameraController"), "Initializing TestT265CameraController...");
                return CallbackReturn::SUCCESS;
            }

            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override {
                RCLCPP_INFO(rclcpp::get_logger("TestT265CameraController"), "Configuring TestT265CameraController...");
                pose_publisher_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "test_t265_pose", 10);
                return CallbackReturn::SUCCESS;
            }

            // 获取状态接口配置
            controller_interface::InterfaceConfiguration state_interface_configuration() const override {
                controller_interface::InterfaceConfiguration config;
                config.type = controller_interface::interface_configuration_type::ALL; 
                return config;
            }

            // 获取指令接口配置
            controller_interface::InterfaceConfiguration command_interface_configuration() const override {
                return {controller_interface::interface_configuration_type::NONE};
            }


            controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override {
                // RCLCPP_INFO(rclcpp::get_logger("TestT265CameraController"), "Updating TestT265CameraController...");
                
                for(const auto& interface : state_interfaces_) {
                    if(interface.get_interface_name() == "data_ptr") {
                        auto* t265_camera_data = reinterpret_cast<perception_hardware::T265CameraData*>(
                            static_cast<uintptr_t>(interface.get_value())
                        );

                        if (t265_camera_data) {

                            auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
                            msg->header.stamp = time;
                            msg->header.frame_id = "test_odom";
                            msg->pose.position.x = t265_camera_data->pose.position[0];
                            msg->pose.position.y = t265_camera_data->pose.position[1];
                            msg->pose.position.z = t265_camera_data->pose.position[2];
                            pose_publisher_->publish(std::move(msg));
                        }                        
                    }
                }
                return controller_interface::return_type::OK;
            }
    };

} // namespace perception_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(perception_controller::TestT265CameraController, controller_interface::ControllerInterface)