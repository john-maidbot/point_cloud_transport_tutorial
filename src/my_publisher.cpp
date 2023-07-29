// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <point_cloud_transport/point_cloud_transport.h>

// for reading rosbag
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("point_cloud_publisher");

  point_cloud_transport::PointCloudTransport pct(node);
  point_cloud_transport::Publisher pub = pct.advertise("pct/point_cloud", 100);

  std::string bagged_cloud_topic_;
  std::string bag_file_;

  rosbag2_cpp::Reader reader;
  reader.open(bag_file_);

  sensor_msgs::msg::PointCloud2 cloud_msg;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
  rcutils_time_point_value_t cloud_time;
  while (reader.has_next() && rclcpp::ok())
  {
    // get serialized data
    auto serialized_message = reader.read_next();      
    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);
    if (serialized_message->topic_name == bagged_cloud_topic_)
    {
      // deserialize and convert to ros2 message
      cloud_serialization.deserialize_message(&extracted_serialized_msg, &cloud_msg);
      pub.publish(cloud_msg);
      rclcpp::spin_some();
    }
  }
  reader.close();

  rclcpp::shutdown();
}
