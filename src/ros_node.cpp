#include "ros_node.h"
#include <sensor_msgs_ext/proximity.h>

ros_node::ros_node(driver* device_driver, int argc, char **argv)
{
    // Take ownership of the device driver.
    ros_node::m_driver = device_driver;

    // Initialize the node.
    ros::init(argc, argv, "driver_proximity");

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
    private_node.param<int>("radiation_type", ros_node::p_radiation_type, 255);
    private_node.param<float>("min_range", ros_node::p_min_range, std::numeric_limits<float>::quiet_NaN());
    private_node.param<float>("max_range", ros_node::p_max_range, std::numeric_limits<float>::quiet_NaN());
    private_node.param<float>("field_of_view", ros_node::p_fov, std::numeric_limits<float>::quiet_NaN());

    // Set up the publisher.
    ros_node::m_publisher = ros_node::m_node->advertise<sensor_msgs_ext::proximity>("proximity", 10);

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
        try
        {
            sensor_msgs_ext::proximity message;
            // Populate header.
            message.frame_id = ros::this_node::getName();
            // Populate sensor characteristics.
            message.radiation_type = static_cast<unsigned char>(ros_node::p_radiation_type);
            message.min_range = ros_node::p_min_range;
            message.max_range = ros_node::p_max_range;
            message.field_of_view = ros_node::p_fov;
            // Populate state.
            // Use XOR to invert reading if necessary.
            message.proximity = ros_node::p_invert_output ^ ros_node::m_driver->read_state();
            ros_node::m_publisher.publish(message);
        }
        catch(std::exception& e)
        {
            ROS_WARN_STREAM(e.what());
        }

        ros_node::m_rate->sleep();
    }
}
