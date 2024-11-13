#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>

using namespace std::chrono_literals;

constexpr uint8_t kResolution = 8; // 8x8
constexpr float kMillimeterToMeter = 1.0f / 1000.0f; // mm to m
constexpr float kFov = M_PI / 4.0f; // 45度のFoV
constexpr float kFovPerPixel = kFov / kResolution; // 各ピクセルのFoV

class PointCloudPublisher : public rclcpp::Node
{
public:
  PointCloudPublisher()
  : Node("multiarray_to_pointcloud"), _debug(false)
  {
    // パラメータ設定
    this->declare_parameter<bool>("debug", false);
    this->get_parameter("debug", _debug);

    // サブスクリプションとパブリッシャーの初期化
    _subscription = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/serial_data", 10, std::bind(&PointCloudPublisher::callback, this, std::placeholders::_1));
    
    _publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("points2", 10);

    if (_debug)
    {
      _marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("tof_debug", 10);
    }
  }

private:
  void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != kResolution * kResolution)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid data size. Expected 64 values.");
      return;
    }

    auto point_cloud = sensor_msgs::msg::PointCloud2();
    point_cloud.header.frame_id = "tof_link";
    point_cloud.header.stamp = this->get_clock()->now();
    point_cloud.height = kResolution;
    point_cloud.width = kResolution;
    point_cloud.is_dense = false;
    point_cloud.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
    pcd_modifier.resize(point_cloud.height * point_cloud.width);

    sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");

    for (int w = kResolution - 1; w >= 0; --w)
    {
      for (size_t h = 0; h < kResolution; ++h)
      {
        float depth = msg->data[w + (kResolution * h)];

        if (depth <= 0.0f)
        {
          *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
        }
        else
        {
          *iter_x = kMillimeterToMeter * cos(w * kFovPerPixel - kFov / 2.0f - ((90.0f * M_PI) / 180.0f) ) * depth;
          *iter_y = kMillimeterToMeter * sin(h * kFovPerPixel - kFov / 2.0f) * depth;
          *iter_z = kMillimeterToMeter * depth;
        }

        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
    }

    _publisher->publish(point_cloud);

    if (_debug)
    {
      publish_marker();
    }
  }

  // デバッグ用マーカーの作成とパブリッシュ
  void publish_marker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "tof_link";
    marker.header.stamp = rclcpp::Time();
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.012;
    marker.scale.z = 0.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    geometry_msgs::msg::Point pt;
    marker.points.push_back(pt);
    pt.z = 0.1;
    marker.points.push_back(pt);

    _marker_publisher->publish(marker);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr _subscription;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _marker_publisher;

  bool _debug;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}

