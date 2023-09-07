// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

// This file is a simple example of how to use the C++ "PointCloudCodec" API of
// point_cloud_transport to encode/decode point clouds without needing to
// publish/subscribe to a topic.

// for ros2 logging + reading rosbag
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>

#include <point_cloud_transport/point_cloud_codec.hpp>

int main(int /*argc*/, char ** /*argv*/)
{
  auto logger_ = rclcpp::get_logger("my_encoder");

  point_cloud_transport::PointCloudCodec codec;

  // set the transport to use
  const std::string transport = "draco";
  RCLCPP_INFO(logger_, "Using transport: %s", transport.c_str());

  const std::string bagged_cloud_topic = "/point_cloud";
  const std::string shared_directory = ament_index_cpp::get_package_share_directory(
    "point_cloud_transport_tutorial");
  const std::string bag_file = shared_directory + "/resources/rosbag2_2023_08_05-16_08_51";

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
  while (reader.has_next()) {
    // get serialized data
    auto serialized_message = reader.read_next();
    rclcpp::SerializedMessage extracted_serialized_msg(*serialized_message->serialized_data);
    if (serialized_message->topic_name == bagged_cloud_topic) {
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

      if (encode_success) {
        RCLCPP_INFO(
          logger_, "ENCODE Raw size: %zu, compressed size: %zu, ratio: %.2f %%, transport type: %s",
          original_serialized_size, compressed_msg.size(),
          100.0 * compressed_msg.size() / original_serialized_size,
          transport.c_str());
      } else {
        RCLCPP_ERROR(logger_, "Failed to encode message");
      }

      //
      // Decode using C++ API
      //
      sensor_msgs::msg::PointCloud2 decoded_msg;
      const bool decode_success = codec.decode(transport, compressed_msg, decoded_msg);
      if (decode_success) {
        RCLCPP_INFO(
          logger_, "DECODE Raw size: %zu, compressed size: %zu, ratio: %.2f %%, transport type: %s",
          original_deserialized_size, decoded_msg.data.size(),
          100.0 * decoded_msg.data.size() / original_deserialized_size,
          transport.c_str());
      } else {
        RCLCPP_ERROR(logger_, "Failed to decode message");
      }
    }
  }
  reader.close();
}
