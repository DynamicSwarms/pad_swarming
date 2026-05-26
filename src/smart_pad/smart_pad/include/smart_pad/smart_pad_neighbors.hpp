#pragma once


#include "rclcpp/rclcpp.hpp"

#include "smart_pad/smart_pad_tf.hpp"



class SmartPadNeighbors
{    
public:
    SmartPadNeighbors(
        std::string pad_name,
        std::shared_ptr<SmartPadTF> smart_pad_tf,
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
        rclcpp::Logger parent_logger)
    : m_pad_name(pad_name)
    , m_smart_pad_tf(smart_pad_tf)
    , m_logger(parent_logger.get_child("SmartPadNeighbors"))
    , p_neighbor_distance_threshold(node_param_interface->declare_parameter("neighbor_distance_threshold", rclcpp::ParameterValue(1.0)).get<double>())
    {
        m_param_callback_handle = node_param_interface->add_on_set_parameters_callback(
            std::bind(&SmartPadNeighbors::m_on_parameters_set, this, std::placeholders::_1)
        );
    }

    bool 
    get_neighbors(std::vector<std::string> & neighbors) {
        neighbors.clear();
        std::vector<std::string> potential_neighbors = m_smart_pad_tf->get_other_smart_pads();
        
        Eigen::Affine3d other_pose_in_my_frame;
        for (const auto & neighbor : potential_neighbors) {
            if (m_smart_pad_tf->get_affine3d_off_in_frame(other_pose_in_my_frame, neighbor, m_pad_name)) {
                double distance = other_pose_in_my_frame.translation().norm();
                if (distance < p_neighbor_distance_threshold) {
                    neighbors.push_back(neighbor);
                }
            }
        }
        return true;
    }
            


        

    rcl_interfaces::msg::SetParametersResult m_on_parameters_set(const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto & param : parameters) {
            if (param.get_name() == "neighbor_distance_threshold") {
                p_neighbor_distance_threshold = param.as_double();
            }
        }
        return result;
    }

private:
    std::string m_pad_name;
    std::shared_ptr<SmartPadTF> m_smart_pad_tf;
    rclcpp::Logger m_logger;

    double p_neighbor_distance_threshold;
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> m_param_callback_handle; 
};
