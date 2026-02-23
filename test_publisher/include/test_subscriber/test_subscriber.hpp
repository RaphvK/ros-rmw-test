#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


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
  void topicCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

 private:

  /**
   * @brief Subscriber
   */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  
};


}
