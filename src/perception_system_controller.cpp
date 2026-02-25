#include "perception_system_controller/perception_system_controller.hpp"
#include "mock_device/mock_camera_data.hpp"
#include "controller_interface/helpers.hpp"

#include <map>

namespace perception_controller {
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

        members_.clear();

        for(auto const& [name, member] : params_.members.device_list_map)
        {   
            auto member_ptr = std::make_shared<BaseMember>();
            member_ptr->name = name;
            member_ptr->device_type = member.device_type;
            member_ptr->enable = member.enable;
            member_ptr->sensor_components = member.sensor_components;
            member_ptr->interface_name = member.interface_name;

            member_ptr->frame_id = member.frame_id;

            for(const auto & sensor_component : member.sensor_components) {
                size_t pos = sensor_component.find("_");
                if (pos == std::string::npos || pos == sensor_component.size() - 1) {
                    RCLCPP_ERROR(get_node()->get_logger(),"Invalid sensor_component format: %s", sensor_component.c_str());
                    continue;
                }
                std::string type = sensor_component.substr(0, pos);
                std::string topic = sensor_component.substr(pos + 1);

                if (type == "image")
                {
                    member_ptr->pub_raw[topic] = get_node()->create_publisher<sensor_msgs::msg::Image>(topic, 10);
                    // member_ptr->pub_rect[topic] = get_node()->create_publisher<sensor_msgs::msg::Image>(topic, 10);
                }
                else if (type == "pose")
                {
                    member_ptr->pub_pose[topic] = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
                }
                else if (type == "imu")
                {
                    member_ptr->pub_imu[topic] = get_node()->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
                }            
                
            }
            members_.push_back(member_ptr);

            RCLCPP_INFO(get_node()->get_logger(), "PerceptionSystemController on_configure member: %s", name.c_str());
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PerceptionSystemController::on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(get_node()->get_logger(), "PerceptionSystemController on_activate");
        is_running_ = true;

        for(auto & member : members_) {

            RCLCPP_INFO(get_node()->get_logger(), "Looking for: [%s], Enable status: %s", 
                member->interface_name.c_str(), member->enable ? "true" : "false");

            for (const auto & si : state_interfaces_) {
                RCLCPP_INFO(get_node()->get_logger(), "Available interface in controller: [%s]", 
                            si.get_name().c_str());
            }
            auto it = std::find_if(
                state_interfaces_.begin(),
                state_interfaces_.end(),
                [&](const hardware_interface::LoanedStateInterface & si) {
                    return si.get_name() == member->interface_name;
                });
            
            if(it != state_interfaces_.end()) {
                size_t idx = std::distance(state_interfaces_.begin(), it);

                if(member->enable) {
                    RCLCPP_INFO(get_node()->get_logger(), "HERE!!!");
                    for(const auto & i : member->pub_raw) {
                        threads_.emplace_back(
                            &PerceptionSystemController::worker_thread, 
                            this, member, idx, "image", i.first);         
                    }
                    for(const auto & i : member->pub_pose) {
                        threads_.emplace_back(
                            &PerceptionSystemController::worker_thread, 
                            this, member, idx, "pose", i.first);         
                    }
                    for(const auto & i : member->pub_imu) {
                        threads_.emplace_back(
                            &PerceptionSystemController::worker_thread, 
                            this, member, idx, "imu", i.first);         
                    }

                    RCLCPP_INFO(get_node()->get_logger(), 
                        "PerceptionSystemController on_activate member: %s", 
                        member->name.c_str());
                }
            }            
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PerceptionSystemController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        
        for(const auto & member : members_)
        {
            config.names.push_back(member->interface_name);
        }
        return config;
    }

    controller_interface::InterfaceConfiguration PerceptionSystemController::command_interface_configuration() const
    {
        return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE,
        };
    }

    controller_interface::CallbackReturn PerceptionSystemController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        is_running_ = false;
        for(auto & thread : threads_) {
            if(thread.joinable()) {
                thread.join();
            }
        }
        threads_.clear();
        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type PerceptionSystemController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        return controller_interface::return_type::OK;
    }

    void PerceptionSystemController::worker_thread(
        std::shared_ptr<BaseMember> member, 
        size_t interface_idx, 
        std::string type, 
        std::string topic)
    {
        auto logger = get_node()->get_logger();
        uint64_t last_processed_timestamp = 0;
        
        while(is_running_ && rclcpp::ok())
        {
            // get raw ptr
            double raw_val = state_interfaces_[interface_idx].get_value();
            uintptr_t ptr_address = static_cast<uintptr_t>(raw_val);
            RCLCPP_INFO(logger, "Get raw ptr: %p, type: %s", 
                reinterpret_cast<void*>(ptr_address), type.c_str());

            if (ptr_address == 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            if(type == "image"){
                RCLCPP_INFO(logger, "Pub image: %s, type: %s", topic.c_str(), type.c_str());
            }
            else if(type == "pose"){
                RCLCPP_INFO(logger, "Pub pose: %s, type: %s", topic.c_str(), type.c_str());
            }
            else if(type == "imu"){
                RCLCPP_INFO(logger, "Pub imu: %s, type: %s", topic.c_str(), type.c_str());
            }
        }
    }



} // namespace perception_controller
