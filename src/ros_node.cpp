#include "ros_node.h"
#include <std_msgs/Bool.h>

ros_node::ros_node(driver* device_driver, int argc, char **argv)
{
    // Take ownership of the device driver.
    ros_node::m_driver = device_driver;

    // Initialize the node.
    ros::init(argc, argv, "driver_gp2y0d8");

    // Get the node's handle.
    // All devices should publish to generic device namespaces.
    ros_node::m_node = new ros::NodeHandle("proximity");

    // Read parameters.
    ros::NodeHandle private_node("~");
    int param_gpio;
    private_node.param<int>("gpio_pin", param_gpio, 0);
    double param_publish_rate;
    private_node.param<double>("publish_rate", param_publish_rate, 30);
    private_node.param<bool>("invert_output", ros_node::p_invert_output, false);

    // Set up the publisher.
    ros_node::m_publisher = ros_node::m_node->advertise<std_msgs::Bool>("proximity", 10);

    // Set the publishing rate.
    ros_node::m_rate = new ros::Rate(param_publish_rate);

    // Initialize the driver.
    try
    {
        ros_node::m_driver->initialize(static_cast<unsigned int>(param_gpio));
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
        // Use XOR to invert reading if necessary.
        message.data = ros_node::p_invert_output ^ ros_node::m_driver->read_state();
        ros_node::m_publisher.publish(message);
        ros_node::m_rate->sleep();
    }
}
