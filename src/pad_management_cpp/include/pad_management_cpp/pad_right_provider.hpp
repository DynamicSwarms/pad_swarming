#pragma once

#include <memory>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pad_management_cpp/I_pad_resource_manager.hpp"
#include "pad_management_cpp/pad_right_server.hpp"

class PadRightActionServerNode : public rclcpp::Node
{
public:
  explicit PadRightActionServerNode(const rclcpp::NodeOptions & options);

private:
  std::shared_ptr<IPadResourceManager> create_pad_resource_manager();

private:
  pluginlib::ClassLoader<IPadResourceManager> m_pad_resource_manager_loader;
  std::shared_ptr<IPadResourceManager> m_pad_resource_manager;
  std::shared_ptr<PadRightServer> m_pad_right_server;
};