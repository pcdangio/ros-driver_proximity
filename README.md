# driver_quad_encoder

## Overview

This package includes driver software for generic digital proximity sensors connected to a GPIO pin.

**Keywords:** proximity driver raspberry_pi

### License

The source code is released under a [MIT license](LICENSE).

**Author: Paul D'Angio<br />
Maintainer: Paul D'Angio, pcdangio@gmail.com**

The driver_proximity package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [sensor_msgs_ext](https://github.com/pcdangio/ros-sensor_msgs_ext) (extension of sensor_msgs for encoders)
- [pigpio](http://abyz.me.uk/rpi/pigpio/) (Raspberry PI I/O)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

        cd catkin_workspace/src
        git clone https://github.com/pcdangio/ros-driver_proximity.git driver_proximity
        cd ../
        catkin_make

## Usage

Run any of the driver nodes with (where xxx is the driver type):

        rosrun driver_proximity xxx_node

For example, to run the node using a driver for a Raspberry Pi:

        rosrun driver_proximity rpi_node

## Nodes

### rpi_node

A Raspberry Pi driver for a digital proximity sensor.  Ensure that the pigpio daemon is running before starting this node.

#### Published Topics
* **`proximity/proximity`** ([sensor_msgs_ext/Proximity])
        The current measurement given by the proximity sensor.


#### Parameters

* **`~/gpio_pin`** (int, default: 0)

        The GPIO input pin connected to the digital proximity sensor.

* **`publish_rate`** (double, default: 30)

        The rate (in Hz) to publish ProximityState messages.

* **`~/invert_output`** (bool, default: FALSE)

        Indicates if the digital reading of the proximity sensor should be inverted.

* **`~/radiation_type`** (uint8, default: UNSPECIFIED)

        Indicates the sensor's radiation type.  See [sensor_msgs_ext/Proximity] for enumerations.

* **`~/min_range`** (float, default: NaN)

        Indicates the minimum detection range of the sensor in meters.

* **`~/max_range`** (float, default: NaN)

        Indicates the maximum detection range of the sensor in meters.

* **`~/field_of_view`** (float, default: NaN)

        Indicates the sensor's maximum field of view in radians.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/pcdangio/ros-driver_proximity/issues).


[ROS]: http://www.ros.org
[sensor_msgs_ext/Proximity]: https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/Proximity.msg
