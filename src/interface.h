/// \file interface.h
/// \brief Defines the interface class.
#ifndef INTERFACE_H
#define INTERFACE_H

#include <stdexcept>

///
/// \brief The hardware abstracted base definition of the GP2Y0D8's hardware interface.
///
class interface
{
public:
    ///
    /// \brief ~interface Pure virtual destructor.
    ///
    virtual ~interface() = 0;

    ///
    /// \brief initialize Initializes the GPIO input pin that the GP2Y0D8 is connected to.
    /// \param gpio_pin The GPIO input pin connected to the GP2Y0D8.
    ///
    virtual void initialize(unsigned int gpio_pin) = 0;

    ///
    /// \brief read_state Reads the immediate state of the GP2Y0D8's input pin.
    /// \return Returns TRUE if the pin is in a high state, otherwise FALSE.
    ///
    virtual bool read_state() = 0;
};

#endif // INTERFACE_H
