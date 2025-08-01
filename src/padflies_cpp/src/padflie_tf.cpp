#include "padflies_cpp/padflie_tf.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;


PadflieTF::PadflieTF(
    const std::string & cf_name,
    const std::string & world_frame)
: m_cf_name(cf_name) 
, m_world_frame(world_frame)
, m_has_pad(false)
, m_last_position()
, m_last_position_time(rclcpp::Time(0))
, m_position_timeout(rclcpp::Duration::from_seconds(1.0))
, m_tf_buffer(std::make_unique<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)))
{
    RCLCPP_INFO(rclcpp::get_logger("TF"), "PadflieTF initialized for CF: %s, world frame: %s", 
                m_cf_name.c_str(), m_world_frame.c_str());      
}

PadflieTF::~PadflieTF()
{
    RCLCPP_INFO(rclcpp::get_logger("TF"), "PadflieTF destructor called");
}

void PadflieTF::on_configure(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    m_node = node;
    m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer, node);
    
    RCLCPP_INFO(node->get_logger(), "PadflieTF creating stufff");      

    m_cf_positions_sub = node->create_subscription<crazyflie_interfaces::msg::PoseStampedArray>(
       "/cf_positions",
       10, 
       std::bind(&PadflieTF::cf_positions_callback, this, _1));
}

void PadflieTF::set_pad(const std::string & pad_name)
{
    m_pad_name = pad_name;
    m_has_pad = true;
}

bool PadflieTF::get_pad_position_and_yaw_or_timeout(
    rclcpp::Duration & timeout_sec,
    Eigen::Vector3d & position,
    double & yaw)
{
    if (!m_has_pad) {
        log("Does not have a pad.");
        return false;
    }

    rclcpp::Time start_time = get_now();
    while (!get_pad_position_and_yaw(position, yaw)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (start_time + timeout_sec < get_now()) {
            log("Timeout while waiting for pad position.");
            return false;
        }
    }

    return true;
}

bool PadflieTF::get_pad_position_and_yaw(
    Eigen::Vector3d & position,
    double & yaw)
{
    if (!m_has_pad) {
        log("Does not have a pad.");
        return false;
    }

    geometry_msgs::msg::TransformStamped transform;
    if (lookup_transform(m_world_frame, m_pad_name, transform))
    {
        position = Eigen::Vector3d(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);
        yaw = tf2::getYaw(transform.transform.rotation);
        return true;
    }
    return false;
}


bool PadflieTF::get_pad_pose(geometry_msgs::msg::PoseStamped & pose_stamped)
{
    if (!m_has_pad) {
        log("Does not have a pad.");
        return false;
    }
    pose_stamped = geometry_msgs::msg::PoseStamped();
    pose_stamped.header.frame_id = m_pad_name;
    return true;
}



bool PadflieTF::get_pad_pose_world(geometry_msgs::msg::PoseStamped & pose_stamped)
{
    if (!m_has_pad) {
        log("Does not have a pad.");
        return false;
    }

    geometry_msgs::msg::TransformStamped transform;

    if (lookup_transform(m_world_frame, m_pad_name, transform))
    {
        geometry_msgs::msg::PoseStamped zero_pose;
        zero_pose.header.frame_id = m_world_frame;
        tf2::doTransform(zero_pose, pose_stamped, transform);
        return true;
    }
    return false;
}

bool PadflieTF::get_cf_pose_stamped(
    const std::string & frame_id,
    geometry_msgs::msg::PoseStamped & pose_stamped)
{
    geometry_msgs::msg::TransformStamped transform;

    if (m_last_position_time + m_position_timeout > get_now()
        && lookup_transform(frame_id, m_world_frame, transform))
    {
        tf2::doTransform(m_last_position, pose_stamped, transform);    
        return true;
    }
    return false;
}


bool PadflieTF::get_cf_position(Eigen::Vector3d & position)
{
    if (m_last_position_time + m_position_timeout > get_now()) {
        position = Eigen::Vector3d(
            m_last_position.pose.position.x,
            m_last_position.pose.position.y,
            m_last_position.pose.position.z);
        return true;
    }
    return false;
}


bool PadflieTF::transform_point_stamped(
    const geometry_msgs::msg::PointStamped & point,
    const std::string & target_frame,
    geometry_msgs::msg::PointStamped & transformed_point)
{
    geometry_msgs::msg::TransformStamped transform;
    if (lookup_transform(target_frame, point.header.frame_id, transform)) {
        // Transform the point
        tf2::doTransform(point, transformed_point, transform);
        transformed_point.header.frame_id = target_frame;
        transformed_point.header.stamp = get_now();
        return true;
    }
    return false;
}

bool PadflieTF::transform_pose_stamped(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & target_frame,
    geometry_msgs::msg::PoseStamped & transformed_pose)
{
    geometry_msgs::msg::TransformStamped transform;
    if (lookup_transform(target_frame, pose.header.frame_id, transform)) {
        // Transform the pose
        tf2::doTransform(pose, transformed_pose, transform);
        transformed_pose.header.frame_id = target_frame;
        transformed_pose.header.stamp = get_now();
        return true;
    }
    return false;
}

bool PadflieTF::pose_stamped_to_world_position_and_yaw(
    const geometry_msgs::msg::PoseStamped & pose_stamped,
    Eigen::Vector3d & position,
    double & yaw)
{
    geometry_msgs::msg::PoseStamped transformed_pose;
    if (transform_pose_stamped(pose_stamped, m_world_frame, transformed_pose)) 
    {
        position = Eigen::Vector3d(
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z);
        yaw = tf2::getYaw(transformed_pose.pose.orientation);
        return true;
    }
    return false;
}

bool PadflieTF::lookup_transform(
    const std::string & target_frame,
    const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & transform)
{
    try {
        transform = m_tf_buffer->lookupTransform(target_frame, source_frame, rclcpp::Time(0));
        return true;
    } catch (const tf2::LookupException & ex) {
        log("LookupException: %s", ex.what());
    } catch (const tf2::ConnectivityException & ex) {
        log("ConnectivityException: %s", ex.what());
    } catch (const tf2::ExtrapolationException & ex) {
       log("ExtrapolationException: %s", ex.what());
    }

    return false;
}

void PadflieTF::cf_positions_callback(
    const crazyflie_interfaces::msg::PoseStampedArray::SharedPtr msg)
{
    for (const auto & pose : msg->poses) {
        if (pose.header.frame_id == m_cf_name) {
            m_last_position = pose;
            m_last_position_time = get_now();
        }
    }
}

template<typename... Args>
void PadflieTF::log(const char* format, Args&&... args)
{
    if (auto node_shared = m_node.lock()) {
        RCLCPP_INFO(node_shared->get_logger(), format, std::forward<Args>(args)...);
    }
}

rclcpp::Time PadflieTF::get_now() 
{
    static rclcpp::Time last_valid_time = rclcpp::Time(0);

    if (auto node_shared = m_node.lock()) {
        last_valid_time = node_shared->now();
    }
    return last_valid_time;
}