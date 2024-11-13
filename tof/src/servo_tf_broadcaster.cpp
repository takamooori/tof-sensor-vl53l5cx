#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h> 

class ServoTFBroadcaster : public rclcpp::Node
{
public:
    ServoTFBroadcaster()
        : Node("servo_tf_broadcaster"),
          tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)),
          current_angle_(0.0)
    {
        // /servo_angle トピックからサーボ角度をサブスクライブ
        angle_subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/servo_angle", 10,
            std::bind(&ServoTFBroadcaster::angleCallback, this, std::placeholders::_1));

        // タイマーをセットして定期的にTFをブロードキャスト
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ServoTFBroadcaster::broadcastTransforms, this));
    }

private:
    void angleCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // 受け取った角度をラジアンに変換して保存
        current_angle_ = msg->data * M_PI / 180.0;
    }

    void broadcastTransforms()
    {
        // servo_linkのTFを作成してブロードキャスト
        geometry_msgs::msg::TransformStamped servo_transform;
        servo_transform.header.stamp = this->get_clock()->now();
        servo_transform.header.frame_id = "base_link";
        servo_transform.child_frame_id = "servo_link";

        servo_transform.transform.translation.x = 0.0;
        servo_transform.transform.translation.y = 0.0;
        servo_transform.transform.translation.z = 0.1;  // サーボの高さを適宜設定

        // サーボの角度に基づく回転を設定（Z軸回転）
        tf2::Quaternion q;
        q.setRPY(0, 0, current_angle_);
        servo_transform.transform.rotation.x = q.x();
        servo_transform.transform.rotation.y = q.y();
        servo_transform.transform.rotation.z = q.z();
        servo_transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(servo_transform);

        // tof_linkのTFを作成してブロードキャスト（servo_linkに固定）
        geometry_msgs::msg::TransformStamped tof_transform;
        tof_transform.header.stamp = this->get_clock()->now();
        tof_transform.header.frame_id = "servo_link";
        tof_transform.child_frame_id = "tof_link";

        tof_transform.transform.translation.x = 0.05;  // ToFセンサーの前後位置を設定
        tof_transform.transform.translation.y = 0.0;
        tof_transform.transform.translation.z = 0.0;

        // ToFセンサーの回転なし
        tof_transform.transform.rotation.x = 0.0;
        tof_transform.transform.rotation.y = 0.0;
        tof_transform.transform.rotation.z = 0.0;
        tof_transform.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(tof_transform);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_subscriber_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    float current_angle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoTFBroadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

