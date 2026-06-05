#include <algorithm>
#include <memory>
#include <string>
#include <cmath>
#include <mutex>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

struct Point
{
  float x;
  float y;
  float z;
};

struct PointWithID
{
  float x;
  float y;
  float z;
  int id;
  double timestamp;
};

class PadTfPublisher : public rclcpp::Node
{
public:
  PadTfPublisher()
  : Node("ID_sort_node")
  {
    declare_parameter<double>("point_buffer_seconds", 0.5);
    buffer_duration_sec_ = get_parameter("point_buffer_seconds").as_double();
    declare_parameter<std::string>("tf_parent_frame", "world");

    parent_frame_ = get_parameter("tf_parent_frame").as_string();

    point_cloud_subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "InitializedDrone",
      10,
      std::bind(&PadTfPublisher::pointCloudCallback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(get_logger(),
      "ID_sort_node listening on [InitializedDrone] and publishing pad TF frames only.");
    RCLCPP_INFO(get_logger(),
      "Crazyflie handling is intentionally ignored for now; only pad positions are published.");
  }

private:
  void publishPadTf(std::size_t pad_index, const Point &position)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = "pad_" + std::to_string(pad_index);
    transform.transform.translation.x = position.x;
    transform.transform.translation.y = position.y;
    transform.transform.translation.z = position.z;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);
    transform.transform.rotation.x = quaternion.x();
    transform.transform.rotation.y = quaternion.y();
    transform.transform.rotation.z = quaternion.z();
    transform.transform.rotation.w = quaternion.w();

    tf_broadcaster_->sendTransform(transform);
  }

  void publishPadTf(std::size_t pad_index, const Point &position, const tf2::Quaternion &quat)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = "pad_" + std::to_string(pad_index);
    transform.transform.translation.x = position.x;
    transform.transform.translation.y = position.y;
    transform.transform.translation.z = position.z;

    transform.transform.rotation.x = quat.x();
    transform.transform.rotation.y = quat.y();
    transform.transform.rotation.z = quat.z();
    transform.transform.rotation.w = quat.w();

    tf_broadcaster_->sendTransform(transform);
  }

  std::vector<int> get_ids_from_buffer()
  {
    std::vector<int> ids;
    for (const auto &point : buffer_) {
        if (std::find(ids.begin(), ids.end(), point.id) == ids.end()) {
            ids.push_back(point.id);
        }
    }
    return ids;
  }

  void remove_all_with_id(int id)
  {
    buffer_.erase(std::remove_if(buffer_.begin(), buffer_.end(),
      [id](const PointWithID &p) { return p.id == id; }), buffer_.end());
  }

  // Group points by id, try to identify pads (3-point constellation), compute orientation and publish TF
  void check_pad()
  {
    std::vector<int> ids = get_ids_from_buffer();
    for (int id : ids) {
        std::vector<PointWithID> points_same_id;
        for (const auto &point : buffer_) {
            if (point.id == id) {
               points_same_id.push_back(point);
            }
        }

        if (points_same_id.size() < 3) {
            continue; // need at least 3 points to define a pad
        } 

        // Use first three points to determine layout
        const auto &A = points_same_id[0];
        const auto &B = points_same_id[1];
        const auto &C = points_same_id[2];

        auto dist2 = [](const PointWithID &p1, const PointWithID &p2) {
            const double dx = p1.x - p2.x;
            const double dy = p1.y - p2.y;
            const double dz = p1.z - p2.z;
            return dx * dx + dy * dy + dz * dz;
        };

        double dAB = dist2(A, B);
        double dBC = dist2(B, C);
        double dAC = dist2(A, C);

        Point front_pt{0.0f, 0.0f, 0.0f};
        Point middle_pt{0.0f, 0.0f, 0.0f};

        if (dAB > dBC && dAB > dAC) {
            front_pt = Point{C.x, C.y, C.z};
            middle_pt = Point{(A.x + B.x) / 2.0f, (A.y + B.y) / 2.0f, (A.z + B.z) / 2.0f};
        } else if (dBC > dAB && dBC > dAC) {
            front_pt = Point{A.x, A.y, A.z};
            middle_pt = Point{(B.x + C.x) / 2.0f, (B.y + C.y) / 2.0f, (B.z + C.z) / 2.0f};
        } else if (dAC > dBC && dAC > dAB) {
            front_pt = Point{B.x, B.y, B.z};
            middle_pt = Point{(A.x + C.x) / 2.0f, (A.y + C.y) / 2.0f, (A.z + C.z) / 2.0f};
        } else {
            continue; // ambiguous
        }

        // calculate orientation (yaw) from middle -> front
        double dx = static_cast<double>(front_pt.x - middle_pt.x);
        double dy = static_cast<double>(front_pt.y - middle_pt.y);
        double length = std::sqrt(dx * dx + dy * dy);
        if (length == 0.0) {
        continue;
        }

        double sin_theta = dy / length;
        sin_theta = std::clamp(sin_theta, -1.0, 1.0);
        double angle_rad = std::asin(sin_theta);
        // Use atan2 for robust yaw
        angle_rad = std::atan2(dy, dx);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, angle_rad);

        Point mid_point = middle_pt;
        RCLCPP_INFO(this->get_logger(), "--- Pad erkannt --- ID: %d, X: %f, Y: %f, Z: %f, Yaw: %f",
                    id, mid_point.x, mid_point.y, mid_point.z, angle_rad * 180.0 / M_PI);

        publishPadTf(static_cast<std::size_t>(id), mid_point, q);
        remove_all_with_id(id);
    }
  }

  void pointCloudCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) {
      RCLCPP_WARN(get_logger(),
        "Ignored InitializedDrone message with %zu values; expected groups of x,y,z,id.",
        msg->data.size());
      return;
    }

    // Expect data layout: [x,y,z,id, x,y,z,id, ...]
    const double now_sec = this->now().seconds();
    std::vector<PointWithID> parsed;
    for (size_t i = 0; i + 3 < msg->data.size(); i += 4) {
      PointWithID p;
      p.x = msg->data[i + 0];
      p.y = msg->data[i + 1];
      p.z = msg->data[i + 2];
      p.id = static_cast<int>(std::lround(msg->data[i + 3]));
      p.timestamp = now_sec;
      parsed.push_back(p);
    }

    if (parsed.empty()) {
      RCLCPP_INFO(get_logger(), "No valid points parsed from InitializedDrone message.");
      return;
    }

    // Append to buffer, prune old and check for pads
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        // prune
        auto it = buffer_.begin();
        while (it != buffer_.end()) {
        if (it->timestamp < now_sec - buffer_duration_sec_) {
            it = buffer_.erase(it);
        } else {
            ++it;
        }
        }
        // append parsed
        for (auto &p : parsed) buffer_.push_back(p);

        check_pad();
    }
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr point_cloud_subscription_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string parent_frame_ = "world";
  std::vector<PointWithID> buffer_;
  std::mutex buffer_mutex_;
  double buffer_duration_sec_ = 0.5;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PadTfPublisher>());
  rclcpp::shutdown();
  return 0;
}
