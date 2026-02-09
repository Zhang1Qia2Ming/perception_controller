#include "controller_interface/controller_interface.hpp"
#include "mock_device/mock_camera_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

namespace perception_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TestMockCameraController : public controller_interface::ControllerInterface {
  private:
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_publishers_;

  public:
  CallbackReturn on_init() override {
    RCLCPP_INFO(get_node()->get_logger(), "Initializing TestMockCameraController...");
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

  // 核心循环逻辑
  controller_interface::return_type update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
    for (const auto & interface : state_interfaces_) {
      if (interface.get_interface_name() == "data_ptr") {
        double raw_val = interface.get_value();
        uintptr_t addr = static_cast<uintptr_t>(raw_val);
        
        // 每秒打印一次，验证地址是否正确传导
        // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        //                      "Controller Link Active! Sensor [%s] data_ptr address: 0x%lx", 
        //                      interface.get_prefix_name().c_str(), addr);
        

        auto* mock_camera_data = reinterpret_cast<perception_hardware::MockCameraData*>(addr);

        // RCLCPP_INFO(get_node()->get_logger(), "Restored Buffer: %p", (void*)(mock_camera_data->image.data));

        if(!mock_camera_data || mock_camera_data->image.empty()) {
          continue;
        }

        uint64_t hw_time = mock_camera_data->timestamp_nanos;
        auto now = get_node()->now().nanoseconds();
        RCLCPP_INFO(get_node()->get_logger(), "Time Delay: %ld ns", now - hw_time);


        const std::string& camera_name = interface.get_prefix_name();

        if(image_publishers_.find(camera_name) == image_publishers_.end()) {
          image_publishers_[camera_name] = get_node()->create_publisher<sensor_msgs::msg::Image>(
            camera_name + "/image_raw", 10);
        }

        //publish image ros2 message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mock_camera_data->image).toImageMsg();
        msg->header.stamp = get_node()->now();
        msg->header.frame_id = mock_camera_data->frame_id;
        image_publishers_[camera_name]->publish(*msg);
      }
    }
    return controller_interface::return_type::OK;
  }
};

} // namespace perception_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(perception_controller::TestMockCameraController, controller_interface::ControllerInterface)