#ifndef _PERCEPTION_SYSTEM_CONTROLLER_BASE_TYPES_HPP_
#define _PERCEPTION_SYSTEM_CONTROLLER_BASE_TYPES_HPP_

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <condition_variable>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// 
#include "perception_hardware/t265_camera_data.hpp"
#include "mock_device/mock_camera_data.hpp"

namespace perception_controller {

struct TopicSync {
        std::mutex mtx;
        std::condition_variable cv;
        bool has_new_data = false;
};

struct BaseMember {
        std::string name;
        std::string device_type;
        std::vector<std::string> sensor_components;
        std::string interface_name;
        std::string frame_id;
        bool enable = false;

        std::map<std::string, uint64_t> last_ts;
        std::unordered_map<std::string, TopicSync> topic_sync_map;
        
        using ImagePub = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr;
        using ImuPub = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr;
        using PosePub = rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr;
        
        // name, publisher
        std::map<std::string, ImagePub> pub_raw;
        std::map<std::string, ImagePub> pub_rect;
        std::map<std::string, ImuPub> pub_imu;
        std::map<std::string, PosePub> pub_pose;
};

struct HighPerformanceMember : public BaseMember {
        // double buffer:
        // todo: 

};

} // namespace perception_controller


#endif // _PERCEPTION_SYSTEM_CONTROLLER_BASE_TYPES_HPP_