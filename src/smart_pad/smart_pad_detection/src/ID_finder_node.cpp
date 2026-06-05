#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <array>
#include <set>
#include <map>
#include <algorithm>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#define Abtastfrequenz 10
#define Sendfrequenz 40
#define StartBits 3
#define UseBits 8
#define CheckBits 8
#define SendBits (StartBits + UseBits + CheckBits) * 2
#define GeneratorPolySize 8

class ObjectTracker : public rclcpp::Node
{
public:
  ObjectTracker() : Node("object_tracker_node")
  {
    this->declare_parameter<std::string>("pc2_topic", "/pointCloud");
    this->declare_parameter<float>("radius", 0.01f);
    this->declare_parameter<double>("latency_threshold", 0.035);

    std::string pc2_topic = this->get_parameter("pc2_topic").as_string();
    radius_ = this->get_parameter("radius").as_double();

    point_cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pc2_topic, 10, std::bind(&ObjectTracker::pc_callback, this, std::placeholders::_1));

    pub_initialized_drone_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("InitializedDrone", 10);

     std::cout << "ID_finder gestartet" << std::endl;
    
  }

private:
  struct Point
  {
    std::array<float, 3> coordinates;
    std::chrono::time_point<std::chrono::system_clock> timestamp;
    int values[1][SendBits] = {};
    bool read = false;
    bool last_read = 0;
    bool new_read = 0;
    int read_counter = 0;
    bool valid = true;

    Point(float x, float y, float z)
        : coordinates({x, y, z}), timestamp(std::chrono::system_clock::now()) {}
  };

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_initialized_drone_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;

  std::vector<Point> points_;
  float radius_;
  int generatorPoly[GeneratorPolySize] = {1, 0, 0, 0, 0, 0, 1, 1};
  int startsignal[6] = {1, 0, 1, 0, 1, 0};

  float distance(const Point &p1, const Point &p2)
  {
    float dx = p2.coordinates[0] - p1.coordinates[0];
    float dy = p2.coordinates[1] - p1.coordinates[1];
    float dz = p2.coordinates[2] - p1.coordinates[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void removePoint(std::vector<Point> &points, const Point &pointToRemove)
  {
    points.erase(std::remove_if(points.begin(), points.end(),
                                [&pointToRemove, this](const Point &p)
                                { return distance(p, pointToRemove) < radius_; }),
                 points.end());
  }

  void removeOldPoints(std::chrono::milliseconds threshold)
  {
    auto now = std::chrono::system_clock::now();
    points_.erase(std::remove_if(points_.begin(), points_.end(),
                                 [now, threshold](const Point &p)
                                 { return std::chrono::duration_cast<std::chrono::milliseconds>(now - p.timestamp) > threshold; }),
                  points_.end());
  }

  bool pointExists(Point &newPoint)
  {
    for (auto &point : points_)
    {
      if (distance(point, newPoint) < radius_)
      {
        point.new_read = 1;
        point.read = 1;
        for (int i = 0; i < 3; ++i)
          point.coordinates[i] = (point.coordinates[i] + newPoint.coordinates[i]) / 2;
        point.timestamp = std::chrono::system_clock::now();
        return true;
      }
    }
    return false;
  }

  void pointReadZero()
  {
    for (auto &point : points_)
    {
      if (point.read)
      {
        point.read = false;
      }
      else
      {
        point.new_read = 0;
      }
    }
  }

  void UpdateRead()
  {
    for (auto &point : points_)
    {
      if (point.last_read != point.new_read)
      {
        for (int j = 0; j < SendBits - 1; ++j)
          point.values[0][j] = point.values[0][j + 1];
        point.values[0][SendBits - 1] = point.new_read;
        point.last_read = point.new_read;
        point.read_counter = 1;
      }
      else
      {
        point.read_counter++;
      }

      if (point.read_counter > Sendfrequenz / Abtastfrequenz * 2 - 1)
      {
        for (int j = 0; j < SendBits - 1; ++j)
          point.values[0][j] = point.values[0][j + 1];
        point.values[0][SendBits - 1] = point.new_read;
        point.read_counter = 0;
      }
    }
  }

  bool CheckStartSignal(Point &point)
  {
    for (int i = 0; i < StartBits; i++)
    {
      if (point.values[0][i * 2] != startsignal[i * 2] || point.values[0][i * 2 + 1] != startsignal[i * 2 + 1])
        return false;
    }
    return true;
  }

  int calculateNumber(Point &point)
  {
    int result = 0;
    for (int i = StartBits; i < StartBits + UseBits; i++)
    {
      if (point.values[0][i * 2] == 1 && point.values[0][i * 2 + 1] == 0)
      {
        result += std::pow(2, StartBits + UseBits - 1 - i);
      }
      else if ((point.values[0][i * 2] == 1 && point.values[0][i * 2 + 1] == 1) ||
               (point.values[0][i * 2] == 0 && point.values[0][i * 2 + 1] == 0))
      {
        point.valid = false;
        break;
      }
    }
    return result;
  }

  std::array<int, 8> calculateCRC(Point &point)
  {
    int reminders[SendBits + CheckBits] = {0};
    std::array<int, GeneratorPolySize> result = {0};

    for (int i = StartBits; i < StartBits + UseBits; i++)
    {
      if (point.values[0][i * 2] == 0 && point.values[0][i * 2 + 1] == 1)
        reminders[i - StartBits] = 0;
      else if (point.values[0][i * 2] == 1 && point.values[0][i * 2 + 1] == 0)
        reminders[i - StartBits] = 1;
      else
        point.valid = false;
    }

    for (int i = 0; i <= UseBits; i++)
    {
      if (reminders[i] == 1 && generatorPoly[0] == 1)
      {
        reminders[i] = 0;
        for (int j = i + 1; j < i + GeneratorPolySize; j++)
        {
          reminders[j] = (generatorPoly[j - i] + reminders[j]) % 2;
        }
      }
    }

    for (int i = 0; i < CheckBits; i++)
      result[i] = reminders[UseBits + i];

    return result;
  }

  std::array<int, 8> getSendCRC(Point &point)
  {
    std::array<int, GeneratorPolySize> result = {0};

    for (int i = 0; i < CheckBits; i++)
    {
      if (point.values[0][(StartBits + UseBits) * 2 + i * 2] == 0 &&
          point.values[0][(StartBits + UseBits) * 2 + i * 2 + 1] == 1)
      {
        result[i] = 0;
      }
      else if (point.values[0][(StartBits + UseBits) * 2 + i * 2] == 1 &&
               point.values[0][(StartBits + UseBits) * 2 + i * 2 + 1] == 0)
      {
        result[i] = 1;
      }
      else
      {
        point.valid = false;
      }
    }
    return result;
  }

  bool checkCRC(const std::array<int, 8> &CRCNumbers, const std::array<int, 8> &SendCRC)
  {
    for (int i = 0; i < CheckBits; i++)
    {
      if (CRCNumbers[i] != SendCRC[i])
        return false;
    }
    return true;
  }

  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    while (iter_x != iter_x.end())
    {
      Point p((*iter_x), (*iter_y), (*iter_z));
      if (!pointExists(p))
      {
        p.new_read = 1;
        p.read = true;
        points_.push_back(p);
      }
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    pointReadZero();
    UpdateRead();
    removeOldPoints(std::chrono::milliseconds(2000));

    for (auto &point : points_)
    {
      point.valid = true;
      if (CheckStartSignal(point) && checkCRC(calculateCRC(point), getSendCRC(point)))
      {
        int id = calculateNumber(point);
        if (point.valid)
        {
          //std::cout << "Gültiger Punkt gefunden: x=" << point.coordinates[0]
          //        << " y=" << point.coordinates[1]
          //        << " z=" << point.coordinates[2]
          //        << " id=" << id << std::endl;
          std_msgs::msg::Float32MultiArray msg_out;
          msg_out.data.push_back(point.coordinates[0]);
          msg_out.data.push_back(point.coordinates[1]);
          msg_out.data.push_back(point.coordinates[2]);
          msg_out.data.push_back(id);
          pub_initialized_drone_->publish(msg_out);
          Point point_to_remove = point;
          removePoint(points_, point_to_remove);
          break;
        }
      }
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectTracker>());
  rclcpp::shutdown();
  return 0;
}

  