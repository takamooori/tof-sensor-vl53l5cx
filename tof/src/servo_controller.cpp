#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <algorithm>  // std::clampを使用するため

class ServoController : public rclcpp::Node {
public:
  ServoController() : Node("servo_controller"), angle_(0), direction_(1) {
    // パブリッシャーの設定
    angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>("servo_angle", 10);
    
    // シリアルポートの設定
    serial_port_ = open("/dev/sensors/servo", O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_port_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
      return;
    }
    set_serial_attributes();

    // タイマーを使って周期的にサーボを動かす
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ServoController::update_servo_angle, this));
  }

  ~ServoController() {
    if (serial_port_ >= 0) {
      close(serial_port_);
    }
  }

private:
  void set_serial_attributes() {
    struct termios tty;
    if (tcgetattr(serial_port_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
      return;
    }
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 1;
    tty.c_cc[VTIME] = 5;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
    }
  }

  void update_servo_angle() {
    angle_ += direction_ * 5;
    if (angle_ >= 180 || angle_ <= 0) {
      direction_ *= -1;
      angle_ = std::clamp(angle_, 0, 180);  // 角度を制限
    }

    // 角度をシリアルポートに送信
    std::string angle_str = std::to_string(static_cast<int>(angle_)) + "\n";
    if (write(serial_port_, angle_str.c_str(), angle_str.length()) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error writing to serial port");
    }

    // 現在の角度をトピックにパブリッシュ
    auto message = std_msgs::msg::Float32();
    message.data = angle_;
    angle_publisher_->publish(message);
  }

  int angle_;
  int direction_;
  int serial_port_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoController>());
  rclcpp::shutdown();
  return 0;
}

