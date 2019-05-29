/// \file rpi_driver.h
/// \brief Defines the rpi_driver class.
#ifndef RPI_DRIVER_H
#define RPI_DRIVER_H

#include "driver.h"

///
/// \brief A Raspberry Pi driver for a digital proximity sensor.
///
class rpi_driver : public driver
{
public:
    ///
    /// \brief rpi_driver Creates a new rpi_driver instance.
    ///
    rpi_driver();
    ///
    /// \brief ~rpi_driver Gracefully closes and destroys the driver.
    ///
    ~rpi_driver() override;

    void initialize(unsigned int gpio_pin) override;
    bool read_state() override;

private:
    ///
    /// \brief m_pigpiod_handle Stores a handle to the pigpio daemon connection.
    ///
    int m_pigpiod_handle;
    ///
    /// \brief mInputPin Stores the input pin that the sensor is connected to.
    ///
    unsigned int m_gpio_pin;
};

#endif // RPI_DRIVER_H
