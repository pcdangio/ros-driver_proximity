/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include <ros/ros.h>
#include "interface.h"

///
/// \brief Implements the ROS Node as an interface for a digital proximity sensor.
///
class ros_node
{
public:
    ///
    /// \brief ros_node Initializes the ROS node.
    /// \param device_interface A polymorphic pointer to the device's interface.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ///
    ros_node(interface* device_interface, int argc, char **argv);
    ~ros_node();

    ///
    /// \brief spin Runs the node.
    ///
    void spin();

private:
    ///
    /// \brief m_interface The polymorphic instance of the device's interface.
    ///
    interface* m_interface;
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
};

#endif // ROS_NODE_H
