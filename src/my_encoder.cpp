// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <iostream>

// for reading rosbag
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>

#include <point_cloud_transport/point_cloud_codec.hpp>

int main(int argc, char **argv)
{
  point_cloud_transport::PointCloudCodec codec;

  std::string transport = "draco";
  if (argc > 2)
  {
    transport = argv[2];
  }

  std::string bagged_cloud_topic_;
  std::string bag_file_;
  auto logger_ = rclcpp::get_logger("my_encoder");

  rosbag2_cpp::Reader reader;
  reader.open(bag_file_);

  sensor_msgs::msg::PointCloud2 cloud_msg;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> cloud_serialization;
  while (reader.has_next() && rclcpp::ok())
  {
    // get serialized data
    auto serialized_message = reader.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);
    if (serialized_message->topic_name == bagged_cloud_topic_)
    {
      // deserialize and convert to ros2 message
      const size_t original_serialized_size = extracted_serialized_msg.size();
      cloud_serialization.deserialize_message(&extracted_serialized_msg, &cloud_msg);
      const size_t original_deserialized_size = cloud_msg.data.size();

      //
      // Encode using C++ API
      //

      // encode/decode communicate via a serialized message. This was done to support arbitrary encoding formats and to make it easy to bind
      // to other languages (since the serialized message is simply a uchar buffer and its size)
      rclcpp::SerializedMessage compressed_msg;
      const bool encode_success = codec.encode(transport, cloud_msg, compressed_msg);

      // BUT encodeTyped/decodeTyped are also available if you would rather work with the actual encoded message type
      // (which may vary depending on the transport being used).

      if(encode_success)
      {
        // ->value() is shorthand for .value().value() (unpacking cras::expected, and then cras::optional)
        RCLCPP_INFO(logger_, "ENCODE Raw size: %zu, compressed size: %zu, ratio: %.2f %%, transport type: %s",
                          original_serialized_size, compressed_msg.size(), 100.0 * compressed_msg.size() / original_serialized_size,
                          transport.c_str());
      }

      //
      // Decode using C++ API
      //
      sensor_msgs::msg::PointCloud2 decoded_msg;
      const bool decode_success = codec.decode(transport, compressed_msg, decoded_msg);
      if(decode_success)
      {
        // ->value() is shorthand for .value().value() (unpacking cras::expected, and then cras::optional)
        RCLCPP_INFO(logger_, "DECODE Raw size: %zu, compressed size: %zu, ratio: %.2f %%, transport type: %s",
                          original_deserialized_size, decoded_msg.data.size(), 100.0 * decoded_msg.data.size() / original_deserialized_size,
                          transport.c_str());
      }
    }
  }
  reader.close();
}
