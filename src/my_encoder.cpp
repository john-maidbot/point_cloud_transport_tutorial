// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak

#include <point_cloud_transport/point_cloud_codec.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

int main(int argc, char** argv)
{
  ros::console::initialize();
  ros::Time::init();
  
  point_cloud_transport::PointCloudCodec pct;
  
  std::string transport = "draco";
  if (argc > 2)
    codec = argv[2];
  else
  {
    config.ints.emplace_back();
    config.ints[0].name = "quantization_POSITION";
    config.ints[0].value = 10;
  }
  auto encoder = pct.getEncoderByName(codec);
  if (!encoder)
    return 1;
  
  ROS_INFO("Found encoder: %s", encoder->getTransportName().c_str());
  
  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);

  for (const auto& m: rosbag::View(bag))
  {
    sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
    if (i != nullptr)
    {
      //
      // Encode the message normally via C++ API
      //
      auto rawLen = ros::serialization::serializationLength(*i);
      auto msg = encoder->encode(*i, config);
      if (!msg)  // msg is of type cras::expected; if it evaluates to false, it has .error(), otherwise it has .value()
        ROS_ERROR("%s", msg.error().c_str());
      else if (!msg.value())  // .value() can be nullptr if the encoder did not return anything for this input
        ROS_INFO("No message produced");
      else  // ->value() is shorthand for .value().value() (unpacking cras::expected, and then cras::optional)
        ROS_INFO_THROTTLE(1.0, "ENCODE Raw size: %zu, compressed size: %u, ratio: %.2f %%, compressed type: %s",
                          i->data.size(), msg->value().size(), 100.0 * msg->value().size() / rawLen,
                          msg->value().getDataType().c_str());

      //
      // Decode using C API; this is much more cumbersome, but it allows writing bindings in other languages.
      //
      
      // Convert config to shapeshifter
      dynamic_reconfigure::Config configMsg;
      topic_tools::ShapeShifter configShifter;
      cras::msgToShapeShifter(configMsg, configShifter);
      
      // Prepare output allocators; this is a very rough idea, they can probably be written much better.
      // Each dynamically sized output of the C API is exported via calling the allocator with a suitable size and
      // writing the result to the allocated buffer.
      sensor_msgs::PointCloud2 raw;
      uint32_t numFields;
      static std::vector<char*> fieldNames;
      fieldNames.clear();
      cras::allocator_t fieldNamesAllocator = [](size_t size) {
        fieldNames.push_back(new char[size]); return reinterpret_cast<void*>(fieldNames.back());};
      static std::vector<uint32_t> fieldOffsets;
      fieldOffsets.clear();
      cras::allocator_t fieldOffsetsAllocator = [](size_t) {
        fieldOffsets.push_back(0); return reinterpret_cast<void*>(&fieldOffsets.back());};
      static std::vector<uint8_t> fieldDatatypes;
      fieldDatatypes.clear();
      cras::allocator_t fieldDatatypesAllocator = [](size_t) {
        fieldDatatypes.push_back(0); return reinterpret_cast<void*>(&fieldDatatypes.back());};
      static std::vector<uint32_t> fieldCounts;
      fieldCounts.clear();
      cras::allocator_t fieldCountsAllocator = [](size_t) {
        fieldCounts.push_back(0); return reinterpret_cast<void*>(&fieldCounts.back());};
      static std::vector<uint8_t> data;
      data.clear();
      cras::allocator_t dataAllocator = [](size_t size) {
        data.resize(size); return reinterpret_cast<void*>(data.data());};
      static char* errorString = nullptr;
      cras::allocator_t errorStringAllocator = [](size_t size) {
        errorString = new char[size]; return reinterpret_cast<void*>(errorString); };

      // Log messages are output as serialized rosgraph_msgs::Log messages.
      static std::vector<cras::ShapeShifter> logMessages;
      cras::allocator_t logMessagesAllocator = [](size_t size) {
        logMessages.emplace_back();
        cras::resizeBuffer(logMessages.back(), size);
        using namespace rosgraph_msgs;
        using namespace ros::message_traits;
        logMessages.back().morph(MD5Sum<Log>::value(), DataType<Log>::value(), Definition<Log>::value(), "0");
        return reinterpret_cast<void*>(cras::getBuffer(logMessages.back()));
      };
      
      // Call the C API
      bool success = pointCloudTransportCodecsDecode(
          "draco", msg->value().getDataType().c_str(), msg->value().getMD5Sum().c_str(),
          cras::getBufferLength(msg->value()), cras::getBuffer(msg->value()), raw.height, raw.width,
          numFields, fieldNamesAllocator, fieldOffsetsAllocator, fieldDatatypesAllocator, fieldCountsAllocator,
          raw.is_bigendian, raw.point_step, raw.row_step, dataAllocator, raw.is_dense,
          cras::getBufferLength(configShifter), cras::getBuffer(configShifter),
          errorStringAllocator, logMessagesAllocator
      );

      // Report all log messages the API call would like to print
      for (const auto& logShifter : logMessages)
      {
        const auto& logMsg = logShifter.instantiate<rosgraph_msgs::Log>();
        ROS_LOG(cras::rosgraphMsgLevelToLogLevel(logMsg->level), ROSCONSOLE_DEFAULT_NAME, "%s", logMsg->msg.c_str());
      }
      
      if (success)
      {
        // Reconstruct the fields
        for (size_t j = 0; j < numFields; ++j)
        {
          sensor_msgs::PointField field;
          field.name = fieldNames[j];
          field.offset = fieldOffsets[j];
          field.datatype = fieldDatatypes[j];
          field.count = fieldCounts[j];
          raw.fields.push_back(field);
        }
        ROS_INFO_THROTTLE(1.0, "DECODE Raw size: %zu, compressed size: %zu, num_fields %zu, field1 %s",
                          data.size(), cras::getBufferLength(msg->value()), fieldNames.size(), fieldNames[0]);
        // ROS_INFO_STREAM_THROTTLE(1.0, raw);
        raw.data = data;
      }
      else
      {
        ROS_ERROR("%s", errorString);
      }
    }
  }
}
