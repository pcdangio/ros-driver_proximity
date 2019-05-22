TEMPLATE = app
CONFIG += console c++11

CONFIG -= qt

HEADERS += \
    src/ros_node.h \
    src/interface.h \
    src/rpi_interface.h

SOURCES += \
    src/main_rpi.cpp \
    src/ros_node.cpp \
    src/interface.cpp \
    src/rpi_interface.cpp

INCLUDEPATH += /opt/ros/melodic/include

DISTFILES += \
    CMakeLists.txt \
    package.xml
