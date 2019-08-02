#include <ros/ros.h>
#include <point_cloud_transport/point_cloud_transport.h>
#include <sensor_msgs/PointCloud2.h>

int message_counter;

void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    message_counter++;
    std::cout << "Message number " << message_counter <<" received, number of points is: " << msg->width*msg->height << std::endl;
}

int main(int argc, char **argv)
{
    message_counter = 0;

    ros::init(argc, argv, "point_cloud_subscriber", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    point_cloud_transport::PointCloudTransport pct(nh);
    point_cloud_transport::Subscriber sub = pct.subscribe("pct/point_cloud", 100, Callback);
    ros::spin();

    return 0;
}

