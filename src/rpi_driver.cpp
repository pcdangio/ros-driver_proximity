#include "rpi_driver.h"

#include <pigpiod_if2.h>

rpi_driver::rpi_driver()
{
    // Connect to the pigpio daemon.
    rpi_driver::m_pigpiod_handle = pigpio_start(nullptr, nullptr);

    // Set default GPIO pin.
    rpi_driver::m_gpio_pin = 0;
}
rpi_driver::~rpi_driver()
{
    // Disconnect from the pigpio daemon.
    pigpio_stop(rpi_driver::m_pigpiod_handle);
}

void rpi_driver::initialize(unsigned int gpio_pin)
{
    // Set the pin as an input pin.
    int result = set_mode(rpi_driver::m_pigpiod_handle, gpio_pin, PI_INPUT);

    // Handle the result.
    switch(result)
    {
    case 0:
    {
        // Success.  Store the gpio pin internally.
        rpi_driver::m_gpio_pin = gpio_pin;
        break;
    }
    case PI_BAD_GPIO:
    {
        throw std::runtime_error("initialize: PIGPIO_D invalid GPIO pin.");
    }
    case PI_BAD_MODE:
    {
        throw std::runtime_error("initialize: PIGPIO_D invalid GPIO mode.");
    }
    case PI_NOT_PERMITTED:
    {
        throw std::runtime_error("initialize: PIGPIO_D not valid.");
    }
    default:
    {
        throw std::runtime_error("initialize: PIGPIO_D unknown error.");
    }
    }
}
bool rpi_driver::read_state()
{
    // Read the state.
    int result = gpio_read(rpi_driver::m_pigpiod_handle, rpi_driver::m_gpio_pin);

    // Handle the result.
    switch(result)
    {
    case 0:
    {
        return false;
    }
    case 1:
    {
        return true;
    }
    case PI_BAD_GPIO:
    {
        throw std::runtime_error("read_state: PIGPIO_D invalid GPIO pin.");
    }
    default:
    {
        throw std::runtime_error("read_state: PIGPIO_D unknown error.");
    }
    }
}
