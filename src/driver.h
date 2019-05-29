/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include <stdexcept>

///
/// \brief The hardware abstracted base definition of the sensor's hardware driver.
///
class driver
{
public:
    ///
    /// \brief ~driver Pure virtual destructor.
    ///
    virtual ~driver() = 0;

    ///
    /// \brief initialize Initializes the GPIO input pin that the sensor is connected to.
    /// \param gpio_pin The GPIO input pin connected to the sensor.
    ///
    virtual void initialize(unsigned int gpio_pin) = 0;

    ///
    /// \brief read_state Reads the immediate state of the sensor's input pin.
    /// \return Returns TRUE if the pin is in a high state, otherwise FALSE.
    ///
    virtual bool read_state() = 0;
};

#endif // DRIVER_H
