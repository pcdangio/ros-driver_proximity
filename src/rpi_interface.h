/// \file rpi_interface.h
/// \brief Defines the rpi_interface class.
#ifndef RPI_INTERFACE_H
#define RPI_INTERFACE_H

#include "interface.h"

///
/// \brief A Raspberry Pi interface for a digital proximity sensor.
///
class rpi_interface : public interface
{
public:
    ///
    /// \brief rpi_interface Creates a new rpi_interface instance.
    ///
    rpi_interface();
    ///
    /// \brief ~rpi_interface Gracefully closes and destroys the interface.
    ///
    ~rpi_interface() override;

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

#endif // RPI_INTERFACE_H
