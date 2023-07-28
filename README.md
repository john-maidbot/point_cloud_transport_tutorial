# \<POINT CLOUD TRANSPORT TUTORIAL>
 **ROS2 v0.1.**

_**Contents**_

  * [Writing a Simple Publisher](#writing-a-simple-publisher)
    * [Code of the Publisher](#code-of-the-publisher)
    * [Code Explained](#code-of-publisher-explained)
    * [Example of Running the Publisher](#example-of-running-the-publisher)
  * [Writing a Simple Subscriber](#writing-a-simple-subscriber)
    * [Code of the Subscriber](#code-of-the-subscriber)
    * [Code Explained](#code-of-subscriber-explained)
    * [Example of Running the Subscriber](#example-of-running-the-subscriber)
  * [Using Publishers And Subscribers With Plugins](#using-publishers-and-subscribers-with-plugins)
    * [Running the Publisher and Subsriber](#running-the-publisher-and-subsriber)
    * [Changing the Transport Used](#changing-the-transport-used)
    * [Changing Transport Behavior](#changing-transport-behavior)
  * [Implementing Custom Plugins](#implementing-custom-plugins)

# Writing a Simple Publisher
In this section, we'll see how to create a publisher node, which opens a .bag file and publishes PointCloud2 messages from it.

This tutorial assumes that you have created your workspace containing [<point_cloud_transport>](https://github.com/john-maidbot/point_cloud_transport)

Before we start, change to the directory and clone this repository:
~~~~~ bash
$ cd ~/<point_cloud_transport_ws>/src
$ git clone git@github.com:john-maidbot/point_cloud_transport_tutorial.git
~~~~~

## Code of the Publisher
Take a look at my_publisher.cpp:
```cpp

```

## Code of Publisher Explained
Now we'll break down the code piece by piece.

Header for including [<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport):
```cpp
#include <point_cloud_transport/point_cloud_transport.h>
```
Headers for opening .bag file with sensor_msgs::PointCloud2 messages:
```cpp
#include <rosbag/bag.h>
#include <rosbag/view.h>
```
Initializing the ROS node:
```cpp
ros::init(argc, argv, "point_cloud_publisher");
ros::NodeHandle nh;
```
Creates *PointCloudTransport* instance and initializes it with our *NodeHandle*. Methods of *PointCloudTransport* can later be used to create point cloud publishers and subscribers similar to how methods of *NodeHandle* are used to create generic publishers and subscribers.
```cpp
point_cloud_transport::PointCloudTransport pct(nh);
```
Uses *PointCloudTransport* method to create a publisher on base topic *"pct/point_cloud"*. Depending on whether more plugins are built, additional (per-plugin) topics derived from the base topic may also be advertised. The second argument is the size of our publishing queue.
```cpp
point_cloud_transport::Publisher pub = pct.advertise("pct/point_cloud", 10);
```
Opens .bag file given as an argument to the program:
```cpp
rosbag::Bag bag;
bag.open(argv[1], rosbag::bagmode::Read);
```

Publishes sensor_msgs::PointCloud2 message from the specified .bag with frequency of 5Hz:
```cpp
ros::Rate loop_rate(5);
    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
        if (i != nullptr)
        {
            pub.publish(i);
            ros::spinOnce();
            loop_rate.sleep();
        }
        if (!nh.ok())
        {
         break;
        }
    }
```

## Example of Running the Publisher
To run my_publisher.cpp, open terminal in the root of workspace and run the following:
~~~~~ bash
$ catkin_make_isolated
$ source devel_isolated/setup.bash
$ rosrun point_cloud_transport_tutorial publisher_test /path/to/my_bag.bag
~~~~~
Of course, roscore/master must also be running.

# Writing a Simple Subscriber
In this section, we'll see how to create a subscriber node, which receives PointCloud2 messages and prints the number of points in them.

## Code of the Subscriber
Take a look at my_subscriber.cpp:
```cpp
#include <ros/ros.h>
#include <point_cloud_transport/point_cloud_transport.h>
#include <sensor_msgs/PointCloud2.h>

void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::cout << "Message received, number of points is: " << msg->width*msg->height << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_subscriber", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    point_cloud_transport::PointCloudTransport pct(nh);
    point_cloud_transport::Subscriber sub = pct.subscribe("pct/point_cloud", 100, Callback);
    ros::spin();

    return 0;
}
```
## Code of Subscriber Explained
Now we'll break down the code piece by piece.

Header for including [<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport):
```cpp
#include <point_cloud_transport/point_cloud_transport.h>
```
A callback function, which we will bind to the subscriber. Whenever our subscriber receives a message, the Callback function gets executed and number of points in the message (*msg->width * msg->height*) is printed.
```cpp
void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    std::cout << "Message received, number of points is: " << msg->width*msg->height << std::endl;
}
```

Initializes the ROS node:
```cpp
ros::init(argc, argv, "point_cloud_subscriber", ros::init_options::AnonymousName);
ros::NodeHandle nh;
```
Creates *PointCloudTransport* instance and initializes it with our *NodeHandle*. Methods of *PointCloudTransport* can later be used to create point cloud publishers and subscribers similar to how methods of *NodeHandle* are used to create generic publishers and subscribers.
```cpp
point_cloud_transport::PointCloudTransport pct(nh);
```
Uses *PointCloudTransport* method to create a subscriber on base topic *"pct/point_cloud"*. The second argument is the size of our subscribing queue. The third argument tells the subscriber to execute Callback function, whenever a message is received.
```cpp
point_cloud_transport::Subscriber sub = pct.subscribe("pct/point_cloud", 100, Callback);
```

## Example of Running the Subscriber
To run my_subscriber.cpp, open terminal in the root of workspace and run the following:
~~~~~ bash
$ catkin_make_isolated
$ source devel_isolated/setup.bash
$ rosrun point_cloud_transport_tutorial subscriber_test
~~~~~
Of course, roscore/master must also be running.

# Using Publishers And Subscribers With Plugins
In this section, we'll first make sure that the nodes are running properly. Later on, we'll change the transport to use Draco compressed format.
## Running the Publisher and Subsriber
Make sure that roscore/master is up and running:
~~~~~ bash
$ roscore
~~~~~
Now we can run the Publisher/Subsriber nodes. To run both start two terminal tabs and enter commands:
~~~~~ bash
$ source devel_isolated/setup.bash
$ rosrun point_cloud_transport_tutorial subscriber_test
~~~~~
And in the second tab:
~~~~~ bash
$ source devel_isolated/setup.bash
$ rosrun point_cloud_transport_tutorial publisher_test /path/to/my_bag.bag
~~~~~
If both nodes are running properly, you should see the subscriber node start printing out messages similar to:
~~~~~ bash
Message received, number of points is: 63744
~~~~~

To list the topics, which are being published and subscribed to, enter command:
~~~~~ bash
$ rostopic list -v
~~~~~
The output should look similar to this:
~~~~~ bash
Published topics:
 * /rosout [rosgraph_msgs/Log] 1 publisher
 * /pct/point_cloud [sensor_msgs/PointCloud2] 1 publisher
 * /rosout_agg [rosgraph_msgs/Log] 1 publisher

Subscribed topics:
 * /rosout [rosgraph_msgs/Log] 1 subscriber
 * /pct/point_cloud [sensor_msgs/PointCloud2] 1 subscriber
~~~~~
To display the ROS computation graph, enter command:

~~~~~ bash
$ rqt_graph
~~~~~
You should see a graph similar to this:

![Graph1](https://github.com/paplhjak/point_cloud_transport_tutorial/blob/master/readme_images/rosgraph1.png)

## Changing the Transport Used
Currently our nodes are communicating raw sensor_msgs/PointCloud2 messages, so we are not gaining anything over using basic ros::Publisher and ros::Subscriber. We can change that by introducing a new transport. 

The [<draco_point_cloud_transport>](https://github.com/paplhjak/draco_point_cloud_transport) package provides plugin for   [<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport), which sends point clouds over the wire encoded through kd-tree or sequantial compression. Notice that draco_point_cloud_transport is not a dependency of your package; point_cloud_transport automatically discovers all transport plugins built in your ROS system.

Assuming you have followed [<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport) [installation](https://github.com/paplhjak/point_cloud_transport#installation), you should already have [<draco_point_cloud_transport>](https://github.com/paplhjak/draco_point_cloud_transport) built.

To check which plugins are built on your machine, enter command:
~~~~~ bash
$ rosrun point_cloud_transport list_transports
~~~~~
You should see output similar to:
~~~~~ bash
Declared transports:
point_cloud_transport/draco
point_cloud_transport/raw

Details:
----------
"point_cloud_transport/draco"
 - Provided by package: draco_point_cloud_transport
 - Publisher: 
      This plugin publishes a CompressedPointCloud2 using KD tree compression.
    
 - Subscriber: 
      This plugin decompresses a CompressedPointCloud2 topic.
    
----------
"point_cloud_transport/raw"
 - Provided by package: point_cloud_transport
 - Publisher: 
            This is the default publisher. It publishes the PointCloud2 as-is on the base topic.
        
 - Subscriber: 
            This is the default pass-through subscriber for topics of type sensor_msgs/PointCloud2.
~~~~~
Shut down your publisher node and restart it. If you list the published topics again and have [<draco_point_cloud_transport>](https://github.com/paplhjak/draco_point_cloud_transport) installed, you should see a new one:

~~~~~ bash
 * /pct/point_cloud/draco [draco_point_cloud_transport/CompressedPointCloud2] 1 publisher
~~~~~

Now let's start up a new subscriber, this one using draco transport. The key is that [<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport) subscribers check the parameter ~point_cloud_transport for the name of a transport to use in place of "raw". Let's set this parameter and start a subscriber node with name "draco_listener":

~~~~~ bash
$ rosparam set /draco_listener/point_cloud_transport draco
$ rosrun point_cloud_transport_tutorial subscriber_test __name:=draco_listener
~~~~~

If we check the node graph again:

~~~~~ bash
$ rqt_graph
~~~~~

![Graph2](https://github.com/paplhjak/point_cloud_transport_tutorial/blob/master/readme_images/rosgraph2.png)

We can see, that draco_listener is listening to a separate topic carrying compressed messages.

Note that if you just want the draco messages, you can change the parameter globally in-line: 

~~~~~ bash
$ rosrun point_cloud_transport_tutorial subscriber_test _point_cloud_transport:=draco
~~~~~

## Changing Transport Behavior
For a particular transport, we may want to tweak settings such as compression level and speed, quantization of particular attributes of point cloud, etc. Transport plugins can expose such settings through rqt_reconfigure. For example, /point_cloud_transport/draco/ allows you to change multiple parameters of the compression on the fly.

For now let's adjust the position quantization. By default, "draco" transport uses quantization of 14 bits, allowing 16384 distinquishable positions in each axis; let's change it to 8 bits (256 positions):

~~~~~ bash
$ rosrun rqt_reconfigure rqt_reconfigure
~~~~~

Now pick /pct/point_cloud/draco in the drop-down menu and move the quantization_POSITION slider down to 8. If you visualize the messages, such as in RVIZ, you should be able to see the level of detail of the point cloud drop.

Dynamic Reconfigure has updated the dynamically reconfigurable parameter /pct/point_cloud/draco/quantization_POSITION. You can verify this by running: 

~~~~~ bash
rosparam get /pct/point_cloud/draco/quantization_POSITION
~~~~~

This should display 8.

Full explanation of the reconfigure parameters and an example of how to use them can be found at [<draco_point_cloud_transport>](https://github.com/paplhjak/draco_point_cloud_transport) repository.


# Implementing Custom Plugins (TODO: john-maidbot, ROS2 port of template coming soon)
The process of implementing your own plugins is described in a separate [repository](https://github.com/john-maidbot/templateplugin_point_cloud_transport).
