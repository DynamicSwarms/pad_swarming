#include "padflies_cpp/commander.hpp"


PadflieCommander::PadflieCommander(
    const std::string & cf_prefix,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_iface)
: m_cf_prefix(cf_prefix)
, m_charge_controller(param_iface)
, m_padflie_tf(cf_prefix, "world")
{
    m_pad_id = param_iface->declare_parameter("pad_id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>();


    m_charge_controller.set_on_charged_callback(
        std::bind(&PadflieCommander::on_charged_callback, this));

    m_padflie_tf.set_pad("pad_" + std::to_string(m_pad_id));

}

bool PadflieCommander::on_configure(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    m_charge_controller.on_configure(m_cf_prefix, node);
    m_padflie_tf.on_configure(node);
    return true; // Indicate successful configuration
}

bool PadflieCommander::on_activate(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    // Activation logic can be added here
    RCLCPP_INFO(node->get_logger(), "Padflie Commander activated for %s", m_cf_prefix.c_str());


    geometry_msgs::msg::PoseStamped pad_pose;
    if (m_padflie_tf.get_pad_pose_world(pad_pose))
    {
        RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Pad position: [x: %.3f, y: %.3f, z: %.3f]", 
                    pad_pose.pose.position.x, pad_pose.pose.position.y, pad_pose.pose.position.z);
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("PadflieCommander"), "Failed to get pad position.");
    }

    m_padflie_actor = std::make_unique<PadflieActor>(node, m_cf_prefix, &m_padflie_tf);

    return true; // Indicate successful activation
}

bool PadflieCommander::on_deactivate(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    RCLCPP_INFO(node->get_logger(), "Padflie Commander deactivated for %s", m_cf_prefix.c_str());
    // Deactivation logic can be added here
    m_padflie_actor.reset(); // Reset the actor to clean up resources
    return true; // Indicate successful deactivation
}

void PadflieCommander::on_charged_callback()
{
    RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Padflie is charged!");
    // Additional logic when the padflie is charged can be added here
}