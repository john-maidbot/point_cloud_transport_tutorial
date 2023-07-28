// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <point_cloud_transport/point_cloud_transport.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_publisher");

  auto node = std::make_shared<rclcpp::Node>();

  point_cloud_transport::PointCloudTransport pct(node);
  point_cloud_transport::Publisher pub = pct.advertise("pct/point_cloud", 100);

  rclcpp::spin(node);

  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);

  ros::Rate loop_rate(5);
  for (const auto& m: rosbag::View(bag))
  {
    sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
    if (i != nullptr)
    {
      pub.publish(i);
      ros::spinOnce();
      loop_rate.sleep();
    }

    if (!ros::ok())
      break;
  }
  rclcpp::shutdown();
}
