/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <ros/ros.h>
#include "driver.h"

///
/// \brief Implements the ROS Node as an driver for a digital proximity sensor.
///
class ros_node
{
public:
    ///
    /// \brief ros_node Initializes the ROS node.
    /// \param device_driver A polymorphic pointer to the device's driver.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ///
    ros_node(driver* device_driver, int argc, char **argv);
    ~ros_node();

    ///
    /// \brief spin Runs the node.
    ///
    void spin();

private:
    ///
    /// \brief m_driver The polymorphic instance of the device's driver.
    ///
    driver* m_driver;
    ///
    /// \brief m_node The node's handle.
    ///
    ros::NodeHandle* m_node;
    ///
    /// \brief m_publisher The node's publisher.
    ///
    ros::Publisher m_publisher;
    ///
    /// \brief m_rate The node's spin rate.
    ///
    ros::Rate* m_rate;
    ///
    /// \brief p_invert_output A parameter indicating if the output of the sensor should be inverted.
    ///
    bool p_invert_output;
};

#endif // ROS_NODE_H
