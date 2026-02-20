#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>


namespace test_subscriber {



/**
 * @brief TestSubscriber class
 */
class TestSubscriber : public rclcpp::Node {

 public:

  TestSubscriber();

 private:

  int msg2_data_;

  /**
   * @brief Sets up subscribers, publishers, etc. to configure the node
   */
  void setup();

  /**
   * @brief Processes messages received by a subscriber
   *
   * @param msg message
   */
  void topicCallback(const std_msgs::msg::UInt8MultiArray::ConstSharedPtr& msg);

 private:

  /**
   * @brief Subscriber
   */
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscriber_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  
};


}
