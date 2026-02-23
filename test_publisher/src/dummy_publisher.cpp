#include <chrono>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

namespace test_publisher {

class DummyPublisher : public rclcpp::Node {
public:
  DummyPublisher() : Node("dummy_publisher"), 
                     rng_(std::random_device{}()),
                     size_dist_(10 * 1024 * 1024, 20 * 1024 * 1024) {
    // Declare and get parameter
    this->declare_parameter("variable_message_size", true);
    variable_message_size_ = this->get_parameter("variable_message_size").as_bool();
    
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "~/topic", 
      1
    );
    timer_ = this->create_wall_timer(
      500ms, std::bind(&DummyPublisher::timerCallback, this));
    RCLCPP_INFO(this->get_logger(), "Publishing to '%s' with %s message size", 
                publisher_->get_topic_name(),
                variable_message_size_ ? "variable (10-20 MB)" : "fixed (10 MB)");
  }

private:
  void timerCallback() {
    auto message = sensor_msgs::msg::PointCloud2();
    // Create message with variable or fixed size
    size_t message_size;
    if (variable_message_size_) {
      message_size = size_dist_(rng_);  // Random between 10-20 MB
    } else {
      message_size = 10 * 1024 * 1024;  // Fixed 10 MB
    }
    
    // Set up PointCloud2 structure
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";
    message.height = 1;
    message.width = message_size / 4;  // Assuming 4 bytes per point (x coordinate as float)
    message.is_bigendian = false;
    message.point_step = 4;
    message.row_step = message_size;
    message.is_dense = true;
    
    message.data.resize(message_size);
    // Fill with pattern based on count
    for (size_t i = 0; i < message.data.size(); i++) {
      message.data[i] = (count_ + i) % 256;
    }
    count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: message #%d (%.2f MB)", count_, (message.data.size() / (1024.0 * 1024.0)));
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  int count_ = 0;
  bool variable_message_size_;
  std::mt19937 rng_;
  std::uniform_int_distribution<size_t> size_dist_;
};

} // namespace test_publisher

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<test_publisher::DummyPublisher>());
  rclcpp::shutdown();
  return 0;
}
