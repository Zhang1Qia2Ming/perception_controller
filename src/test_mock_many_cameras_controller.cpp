#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <map>

#include "controller_interface/controller_interface.hpp"
#include "mock_device/mock_camera_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

namespace perception_controller {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TestMockManyCamerasController : public controller_interface::ControllerInterface {
  private:

    // ROS2 publisher Pool
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_publishers_;
    
    std::thread publish_thread_;
    std::atomic<bool> is_running{false};
    std::condition_variable cv_;
    std::mutex mutex_;

    std::map<std::string, uintptr_t> latest_tasks_;

    // struct PublishTask {
    //   uintptr_t addr;
    //   std::string camera_name;
    // };
    // std::queue<PublishTask> task_queue_;

    void publish_worker() {
      RCLCPP_INFO(get_node()->get_logger(), "Publish worker thread started");
      while(is_running) {
        // PublishTask task;
        std::map<std::string, uintptr_t> current_batch;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          
          cv_.wait(lock, [this]{ return !latest_tasks_.empty() || !is_running;});
          if(!is_running) {
            break;
          }
          current_batch.swap(latest_tasks_);
          // task = task_queue_.front();
          // task_queue_.pop();
        }

        for(auto const& [camera_name, addr] : current_batch) {
          if(addr == 0) continue;

          auto* mock_camera_data = reinterpret_cast<perception_hardware::MockCameraData*>(addr);

          if(!mock_camera_data || mock_camera_data->image.empty()) {
            continue;
          }

          try {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mock_camera_data->image).toImageMsg();
              msg->header.stamp = get_node()->now();
              msg->header.frame_id = mock_camera_data->frame_id;
              
              if(image_publishers_.count(camera_name)) {
                image_publishers_[camera_name]->publish(*msg);      
              }
                                          
          } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Publish image exception: %s", e.what());
          }
        }


        
        // auto* mock_camera_data = reinterpret_cast<perception_hardware::MockCameraData*>(task.addr);
        // auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mock_camera_data->image).toImageMsg();
        // msg->header.stamp = get_node()->now();
        // msg->header.frame_id = mock_camera_data->frame_id;
        // image_publishers_[task.camera_name]->publish(*msg);

      }
    }


  public:
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_node()->get_logger(), "Activating TestMockCameraController...");
    is_running = true;
    publish_thread_ = std::thread(&TestMockManyCamerasController::publish_worker, this);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating TestMockCameraController...");
    is_running = false;
    cv_.notify_all();
    if(publish_thread_.joinable()) {
      publish_thread_.join();
    }
    return CallbackReturn::SUCCESS;
  }

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

        std::string camera_name = interface.get_prefix_name();
        if(image_publishers_.find(camera_name) == image_publishers_.end()) {
          image_publishers_[camera_name] = get_node()->create_publisher<sensor_msgs::msg::Image>(
            camera_name + "/image_raw", 10);
        }
        
        {
          std::lock_guard<std::mutex> lock(mutex_);
          // task_queue_.push({addr, interface.get_prefix_name()});
          latest_tasks_[camera_name] = addr;
        }
        cv_.notify_one();
        // 每秒打印一次，验证地址是否正确传导
        // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        //                      "Controller Link Active! Sensor [%s] data_ptr address: 0x%lx", 
        //                      interface.get_prefix_name().c_str(), addr);
        

        // auto* mock_camera_data = reinterpret_cast<perception_hardware::MockCameraData*>(addr);

        // RCLCPP_INFO(get_node()->get_logger(), "Restored Buffer: %p", (void*)(mock_camera_data->image.data));

        // if(!mock_camera_data || mock_camera_data->image.empty()) {
        //   continue;
        // }

        // uint64_t hw_time = mock_camera_data->timestamp_nanos;
        // auto now = get_node()->now().nanoseconds();
        // RCLCPP_INFO(get_node()->get_logger(), "Time Delay: %ld ns", now - hw_time);


        // const std::string& camera_name = interface.get_prefix_name();

        // if(image_publishers_.find(camera_name) == image_publishers_.end()) {
        //   image_publishers_[camera_name] = get_node()->create_publisher<sensor_msgs::msg::Image>(
        //     camera_name + "/image_raw", 10);
        // }

        //publish image ros2 message
        // auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", mock_camera_data->image).toImageMsg();
        // msg->header.stamp = get_node()->now();
        // msg->header.frame_id = mock_camera_data->frame_id;
        // image_publishers_[camera_name]->publish(*msg);
      }
    }
    return controller_interface::return_type::OK;
  }
};

} // namespace perception_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(perception_controller::TestMockManyCamerasController, controller_interface::ControllerInterface)
