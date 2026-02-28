#ifndef _PERCEPTION_SYSTEM_CONTROLLER_UTILS_HPP_
#define _PERCEPTION_SYSTEM_CONTROLLER_UTILS_HPP_

#include <string>
#include <memory>
#include <atomic>
#include <unordered_map>

//
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

//
#include "perception_hardware/t265_camera_data.hpp"
#include "mock_device/mock_camera_data.hpp"
#include "perception_system_controller/base_types.hpp"

namespace perception_controller {

class DeviceRegistry {
    public:

        static void dispatch(
            uintptr_t ptr,
            const std::string& type,  // image, imu, pose
            const std::string& topic,
            std::shared_ptr<BaseMember> member
        )
        {
            if(member->device_type == "t265_camera") {
                // RCLCPP_INFO(
                //     rclcpp::get_logger("dispatch_t265_camera"),
                //     "dispatch t265 camera %s to %s",
                //     topic.c_str(),
                //     member->name.c_str()
                // );
                process_t265_camera(ptr, type, topic, member);
            }
            else if(member->device_type == "mock_camera") {
                // RCLCPP_INFO(
                //     rclcpp::get_logger("dispatch_mock_camera"),
                //     "dispatch mock camera %s to %s",
                //     topic.c_str(),
                //     member->name.c_str()
                // );
                process_mock_camera(ptr, type, topic, member);
            }
        }

    private:
        static void process_t265_camera(
            uintptr_t ptr,
            const std::string& type,
            const std::string& topic,
            std::shared_ptr<BaseMember> member
        )
        {
            auto* data = reinterpret_cast<perception_hardware::T265CameraData*>(ptr);
            if(data == nullptr) {
                return;
            }

            if(type == "imu") {
                //publish sensor_msgs::msg::Imu
                uint64_t current_ts = data->imu.timestamp_nanos.load(std::memory_order_acquire);
                
                if (current_ts <= member->last_ts[topic]) return;
                member->last_ts[topic] = current_ts;

                auto msg = sensor_msgs::msg::Imu();
                msg.header.stamp = rclcpp::Time(current_ts);
                msg.header.frame_id = member->frame_id;

                // map T265 Left-Hand -> ROS Right-Hand REP-103
                // ROS_X = -T265_Z, ROS_Y = -T265_X, ROS_Z = T265_Y
                msg.linear_acceleration.x = -data->imu.accel[2]; 
                msg.linear_acceleration.y = -data->imu.accel[0];
                msg.linear_acceleration.z =  data->imu.accel[1];

                msg.angular_velocity.x = -data->imu.gyro[2];
                msg.angular_velocity.y = -data->imu.gyro[0];
                msg.angular_velocity.z =  data->imu.gyro[1];

                // add covariance
                msg.linear_acceleration_covariance[0] = 1.e-6;
                msg.linear_acceleration_covariance[4] = 1.e-6;
                msg.linear_acceleration_covariance[8] = 1.e-6;

                msg.angular_velocity_covariance[0] = 1.e-4;
                msg.angular_velocity_covariance[4] = 1.e-4;
                msg.angular_velocity_covariance[8] = 1.e-4;
                
                //if not set Quaternion, 
                //set to -1 to indicate not available (ROS standard)
                msg.orientation_covariance[0] = -1;

                member->pub_imu[topic]->publish(msg);
            }
            else if(type == "pose") {
                //publish geometry_msgs::msg::PoseStamped
                uint64_t current_ts = data->pose.timestamp_nanos.load(std::memory_order_acquire);

                if (current_ts <= member->last_ts[topic]) return;
                member->last_ts[topic] = current_ts;

                // PoseStamped structure:
                //   position: x,y,z
                //   orientation: x,y,z,w
                auto msg = geometry_msgs::msg::PoseStamped();
                msg.header.stamp = rclcpp::Time(current_ts);
                msg.header.frame_id = "odom";

                msg.pose.position.x = -data->pose.translation[2]; 
                msg.pose.position.y = -data->pose.translation[0];
                msg.pose.position.z =  data->pose.translation[1];

                msg.pose.orientation.x = data->pose.rotation[2];
                msg.pose.orientation.y = data->pose.rotation[0];
                msg.pose.orientation.z = data->pose.rotation[1];
                msg.pose.orientation.w = data->pose.rotation[3];

                member->pub_pose[topic]->publish(msg);
            }
            else if(type == "image") {
                //publish sensor_msgs::msg::Image
                //distribute to left or right camera
                bool is_left = (topic.find("left") != std::string::npos || topic.find("0") != std::string::npos);
                auto& frame = is_left ? data->fisheye_left : data->fisheye_right;

                uint64_t current_ts = frame.timestamp_nanos.load(std::memory_order_acquire);
                
                if (current_ts <= member->last_ts[topic]) return;
                member->last_ts[topic] = current_ts;

                if(frame.image.empty()) return;
                
                auto msg = cv_bridge::CvImage(
                    std_msgs::msg::Header(),
                    "mono8",
                    frame.image
                ).toImageMsg();
                msg->header.stamp = rclcpp::Time(current_ts);
                msg->header.frame_id = member->frame_id;

                member->pub_raw[topic]->publish(*msg);
            }
        }

        static void process_mock_camera(
            uintptr_t ptr,
            const std::string& type,
            const std::string& topic,
            std::shared_ptr<BaseMember> member
        )
        {
            // return;
            // RCLCPP_DEBUG(
            //     rclcpp::get_logger("dispatch_mock_camera"),
            //     "dispatch mock camera %s to %s",
            //     topic.c_str(),
            //     member->name.c_str()
            // );
            auto* data = reinterpret_cast<perception_hardware::MockCameraData*>(ptr);
            if(data == nullptr) {
                return;
            }

            if(type == "image") {
                //publish sensor_msgs::msg::Image
                uint64_t current_ts = data->timestamp_nanos.load(std::memory_order_acquire);

                if (current_ts <= member->last_ts[topic]) return;
                member->last_ts[topic] = current_ts;

                if(data->image.empty()){
                    RCLCPP_INFO(
                        rclcpp::get_logger("dispatch_mock_camera"),
                        "mock camera %s timestamp %lu, image empty",
                        topic.c_str(),
                        current_ts
                    );
                    return;
                } 
                auto msg = cv_bridge::CvImage(
                    std_msgs::msg::Header(),
                    "bgr8",
                    data->image
                ).toImageMsg();
                msg->header.stamp = rclcpp::Time(current_ts);
                msg->header.frame_id = member->frame_id;

                member->pub_raw[topic]->publish(*msg);
            }
        }

};

} // namespace perception_controller

#endif // _PERCEPTION_SYSTEM_CONTROLLER_UTILS_HPP_
