#include "ros_node.h"
#include <std_msgs/Bool.h>

ros_node::ros_node(interface* device_interface, int argc, char **argv)
{
    // Take ownership of the device interface.
    ros_node::m_interface = device_interface;

    // Initialize the node.
    ros::init(argc, argv, "interface_gp2y0d8");

    // Get the node's handle.
    // All devices should publish to generic device namespaces.
    ros_node::m_node = new ros::NodeHandle("proximity");

    // Read parameters.
    ros::NodeHandle private_node("~");
    int param_gpio;
    private_node.param<int>("proximity/gpio", param_gpio, 0);
    double param_publish_rate;
    private_node.param<double>("proximity/rate", param_publish_rate, 30);

    // Set up the publisher.
    ros_node::m_publisher = ros_node::m_node->advertise<std_msgs::Bool>("proximity", 10);

    // Set the publishing rate.
    ros_node::m_rate = new ros::Rate(param_publish_rate);

    // Initialize the interface.
    try
    {
        ros_node::m_interface->initialize(param_gpio);
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM(e.what());
        ros::shutdown();
    }
}

ros_node::~ros_node()
{
    delete ros_node::m_rate;
    delete ros_node::m_node;
}

void ros_node::spin()
{
    while(ros::ok())
    {
        std_msgs::Bool message;
        message.data = ros_node::m_interface->read_state();
        ros_node::m_publisher.publish(message);
        ros_node::m_rate->sleep();
    }
}
