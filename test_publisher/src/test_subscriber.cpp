#include <functional>

#include <test_subscriber/test_subscriber.hpp>


namespace test_subscriber {


TestSubscriber::TestSubscriber() : Node("test_subscriber") {

  this->setup();
}


void TestSubscriber::setup() {

  // subscriber for handling incoming messages
  subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input", 
    1,
    std::bind(&TestSubscriber::topicCallback, this, std::placeholders::_1)
  );
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", subscriber_->get_topic_name());
  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "~/output", 
    1
  );
  RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", publisher_->get_topic_name());
}


void TestSubscriber::topicCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {

  RCLCPP_INFO(this->get_logger(), "Message received with size: '%ld'", msg->data.size());
  publisher_->publish(*msg);
}

}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  auto node = std::make_shared<test_subscriber::TestSubscriber>();
  rclcpp::executors::SingleThreadedExecutor executor;
  RCLCPP_INFO(node->get_logger(), "Spinning node '%s' with %s", node->get_fully_qualified_name(), "SingleThreadedExecutor");
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
