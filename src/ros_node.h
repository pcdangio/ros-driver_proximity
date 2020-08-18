/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <sensor_msgs_ext/get_proximity_configuration.h>

#include <ros/ros.h>

/// \brief Implements the ROS Node as an driver for a digital proximity sensor.
class ros_node
{
public:
    // CONSTRUCTORS
    /// \brief Initializes the ROS node.
    /// \param device_driver A polymorphic pointer to the device's driver.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ros_node(driver* device_driver, int argc, char **argv);
    ~ros_node();

    // METHODS
    /// \brief Runs the node.
    void spin();

private:
    // DRIVER
    /// \brief The polymorphic instance of the device's driver.
    driver* m_driver;

    // ROS
    /// \brief The node's handle.
    ros::NodeHandle* m_node;
    /// \brief The node's spin rate.
    ros::Rate* m_rate;

    // PUBLISHERS
    /// \brief Publishes proximity messages.
    ros::Publisher m_publisher;

    // SERVICES
    /// \brief Service for getting the sensor's configuration.
    ros::ServiceServer m_service_get_cfg;
    /// \brief Service callback for m_service_get_cfg.
    /// \param request The service request.
    /// \param response The service response.
    /// \returns TRUE if the service succeeded, otherwise FALSE.
    bool service_get_cfg(sensor_msgs_ext::get_proximity_configurationRequest& request, sensor_msgs_ext::get_proximity_configurationResponse& response);

    // PARAMETERS
    /// \brief A parameter indicating if the output of the sensor should be inverted.
    bool p_invert_output;
    /// \brief A parameter indicating the radiation type of the proximity sensor.
    int p_radiation_type;
    /// \brief A parameter indicating the minimum detection range of the proximity sensor.
    float p_min_range;
    /// \brief A parameter indicating the maximum detection range of the proximity sensor.
    float p_max_range;
    /// \brief A parameter indicating the maximum field of view of the proximity sensor.
    float p_fov;
};

#endif // ROS_NODE_H
