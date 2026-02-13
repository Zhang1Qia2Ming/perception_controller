#ifndef PERCEPTION_CONTROLLER__PERCEPTION_SYSTEM_CONTROLLER_HPP_
#define PERCEPTION_CONTROLLER__PERCEPTION_SYSTEM_CONTROLLER_HPP_

#include <atomic>
#include <thread>
#include <map>
#include <algorithm>
#include <iterator>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <perception_controller/perception_system_controller_params_lib.hpp>


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
        struct BaseMember {
            std::string name;
            std::string device_type;
            std::vector<std::string> sensor_components;
            std::string interface_name;
            std::string frame_id;
            bool enable = false;

            using ImagePub = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr;
            using ImuPub = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr;
            using PosePub = rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr;
            
            ImagePub pub_raw;
            ImagePub pub_rect;
            ImuPub pub_imu;
            PosePub pub_pose;
        };
        void worker_thread(std::shared_ptr<BaseMember> member, size_t interface_idx);
        
        std::vector<std::shared_ptr<BaseMember>> members_;

        std::shared_ptr<ParamListener> param_listener_;
        Params params_;

        // about thread
        std::vector<std::thread> threads_;
        std::atomic<bool> is_running_{false};
};

} // namespace perception_system_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    perception_system_controller::PerceptionSystemController, controller_interface::ControllerInterface)

#endif // PERCEPTION_CONTROLLER__PERCEPTION_SYSTEM_CONTROLLER_HPP_
