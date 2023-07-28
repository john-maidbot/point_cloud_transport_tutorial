// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("point_cloud_subscriber"), "Message received, number of points is: " << msg->width * msg->height);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "point_cloud_subscriber");
  auto node = std::make_shared<rclcpp::Node>();

  point_cloud_transport::PointCloudTransport pct(node);
  point_cloud_transport::Subscriber sub = pct.subscribe("pct/point_cloud", 100, Callback);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}

