// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

// This file is a simple example of how to use the C++ "PointCloudTransport" API of
// point_cloud_transport to publish encoded point_cloud messages via ROS2.

#include <point_cloud_transport/point_cloud_transport.hpp>

#include <iostream>

// for reading rosbag
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcpputils/filesystem_helper.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("point_cloud_publisher");

  point_cloud_transport::PointCloudTransport pct(node);
  point_cloud_transport::Publisher pub = pct.advertise("pct/point_cloud", 100);

  const std::string bagged_cloud_topic = "/point_cloud";
  const std::string shared_directory = ament_index_cpp::get_package_share_directory(
    "point_cloud_transport_tutorial");
  std::string bag_file = shared_directory + "/resources/rosbag2_2023_08_05-16_08_51";

  if (argc > 1)
  {
    bag_file = argv[1];
  }

  if (!rcpputils::fs::exists(bag_file))
  {
    std::cout << "Not able to open file [" << bag_file << "]" << '\n';
    return -1;
  }

  std::cout << "Reading [" << bag_file << "] bagfile" << '\n';

  // boiler-plate to tell rosbag2 how to read our bag
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_file;
  storage_options.storage_id = "mcap";
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  // open the rosbag
  rosbag2_cpp::readers::SequentialReader reader;
  reader.open(storage_options, converter_options);

  sensor_msgs::msg::PointCloud2 cloud_msg;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
  while (reader.has_next() && rclcpp::ok()) {
    // get serialized data
    auto serialized_message = reader.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);
    if (serialized_message->topic_name == bagged_cloud_topic) {
      // deserialize and convert to ros2 message
      cloud_serialization.deserialize_message(&extracted_serialized_msg, &cloud_msg);
      // publish the message
      pub.publish(cloud_msg);
      rclcpp::spin_some(node);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }
  reader.close();

  node.reset();
  rclcpp::shutdown();
}
