// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

// This file is a simple example of how to use the C++ "PointCloudTransport" API of
// point_cloud_transport to subscribe to and decode compressed point_cloud messages via ROS2.

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("point_cloud_subscriber");

  point_cloud_transport::PointCloudTransport pct(node);
  point_cloud_transport::Subscriber pct_sub = pct.subscribe(
    "pct/point_cloud", 100,
    [node](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
    {
      RCLCPP_INFO_STREAM(
        node->get_logger(),
        "Message received, number of points is: " << msg->width * msg->height);
    }, {});

  RCLCPP_INFO_STREAM(node->get_logger(), "Waiting for point_cloud message...");

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
