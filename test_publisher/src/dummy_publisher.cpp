#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

using namespace std::chrono_literals;

namespace test_publisher {

class DummyPublisher : public rclcpp::Node {
public:
  DummyPublisher() : Node("dummy_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
      "~/topic", 
      10
    );
    timer_ = this->create_wall_timer(
      500ms, std::bind(&DummyPublisher::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());
  }

private:
  void timerCallback() {
    auto message = std_msgs::msg::UInt8MultiArray();
    // Create 10 MB message (20 * 1024 * 1024 = 10,485,760 bytes)
    message.data.resize(20 * 1024 * 1024);
    // Fill with pattern based on count
    for (size_t i = 0; i < message.data.size(); i++) {
      message.data[i] = (count_ + i) % 256;
    }
    count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: message #%d (%.2f MB)", count_, (message.data.size() / (1024.0 * 1024.0)));
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  int count_ = 0;
};

} // namespace test_publisher

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<test_publisher::DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
