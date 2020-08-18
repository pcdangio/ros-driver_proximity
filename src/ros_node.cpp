#include "ros_node.h"
#include <sensor_msgs_ext/proximity.h>

// CONSTRUCTORS
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
    int param_gpio = private_node.param<int>("gpio_pin", 0);
    double param_publish_rate = private_node.param<double>("publish_rate", 30);
    ros_node::p_invert_output = private_node.param<bool>("invert_output", false);
    ros_node::p_radiation_type = private_node.param<int>("radiation_type", 255);
    ros_node::p_min_range = private_node.param<float>("min_range", std::numeric_limits<float>::quiet_NaN());
    ros_node::p_max_range = private_node.param<float>("max_range", std::numeric_limits<float>::quiet_NaN());
    ros_node::p_fov = private_node.param<float>("field_of_view", std::numeric_limits<float>::quiet_NaN());

    // Set up the publisher.
    ros_node::m_publisher = ros_node::m_node->advertise<sensor_msgs_ext::proximity>("proximity", 10);

    // Set up the configuration service.
    ros_node::m_service_get_cfg = ros_node::m_node->advertiseService("get_configuration", &ros_node::service_get_cfg, this);

    // Set the publishing rate.
    ros_node::m_rate = new ros::Rate(param_publish_rate);

    // Initialize the driver.
    try
    {
        ros_node::m_driver->initialize(static_cast<unsigned int>(param_gpio));
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM("failed to initialize driver (" << e.what() << ")");
        ros::shutdown();
    }
}
ros_node::~ros_node()
{
    delete ros_node::m_rate;
    delete ros_node::m_node;
}

// METHODS
void ros_node::spin()
{
    while(ros::ok())
    {
        try
        {
            sensor_msgs_ext::proximity message;
            // Populate state.
            // Use XOR to invert reading if necessary.
            message.proximity = ros_node::p_invert_output ^ ros_node::m_driver->read_state();
            ros_node::m_publisher.publish(message);
        }
        catch(std::exception& e)
        {
            ROS_ERROR_STREAM("failed to read state from sensor: (" << e.what() << ")");
        }

        ros_node::m_rate->sleep();
    }
}

// SERVICES
bool ros_node::service_get_cfg(sensor_msgs_ext::get_proximity_configurationRequest& request, sensor_msgs_ext::get_proximity_configurationResponse& response)
{
    // Populate sensor configuration.
    response.radiation_type = static_cast<uint8_t>(ros_node::p_radiation_type);
    response.min_range = ros_node::p_min_range;
    response.max_range = ros_node::p_max_range;
    response.field_of_view = ros_node::p_fov;

    return true;
}