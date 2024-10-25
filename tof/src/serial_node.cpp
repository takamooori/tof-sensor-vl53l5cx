#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <boost/asio.hpp>
#include <sstream>
#include <vector>

using namespace std::chrono_literals;

class SerialNode : public rclcpp::Node
{
public:
    SerialNode() : Node("serial_node"), io(), serial(io)
    {
        // シリアルポートを設定します
        std::string port = "/dev/sensors/tof"; // ポートの名前は適宜変更
        unsigned int baud_rate = 115200;   // ボーレート
        try
        {
            serial.open(port);
            serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        }
        catch (boost::system::system_error& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            return;
        }

        // Publisherの初期化
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("serial_data", 10);
        
        // タイマーの設定 (10msごとにシリアルポートをチェック)
        timer_ = this->create_wall_timer(10ms, std::bind(&SerialNode::readSerialData, this));
    }

private:
    void readSerialData()
{
    if (serial.is_open())
    {
        try
        {
            char c;
            std::string result;

            // シリアルポートからデータを1行読み取る
            while (boost::asio::read(serial, boost::asio::buffer(&c, 1)))
            {
                if (c == '\r') continue;  // キャリッジリターンを無視
                if (c == 'A') continue;   // 'A' を無視
                if (c == '\n') break;     // 改行があればループを抜ける
                result += c;              // 文字を結果に追加
            }

            if (!result.empty())
            {
                // カンマで区切られた数値を処理するための配列
                auto message = std_msgs::msg::Float32MultiArray();
                message.data.clear();
                message.data.resize(64);  // 64個のデータを保持

                // カンマで区切って浮動小数点に変換し、配列に格納
                std::stringstream ss(result);
                std::string token;
                int index = 0;

                while (std::getline(ss, token, ','))
                {
                    if (index < 64)
                    {
                        message.data[index] = std::stof(token);  // トークンをfloatに変換
                        ++index;
                    }
                }

                if (index == 64)  // 64個のデータが揃っているか確認
                {
                    publisher_->publish(message);  // データをパブリッシュ
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Received incomplete data, ignoring...");
                }
            }
        }
        catch (boost::system::system_error& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading serial port: %s", e.what());
        }
    }
}


    boost::asio::io_service io;
    boost::asio::serial_port serial;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialNode>());
    rclcpp::shutdown();
    return 0;
}

