# atr_utils (Vanilla)

This repo contains a ros2 package with the ATR utils C++ libraries, e.g. real-time initialization functions, extended Eigen/TF conversions, etc.

## Description

This package provides dynamic libraries (*.so) with tools and common functions. It is divided in different source files organized by function:

1. atr_utils: Useful functions, e.g. printVector, Basic rotation matrix in z, etc.

1. AuxTools: Provides the AuxTools class with transformation functions, e.g. transform Eigen::Matrix to polygons, etc.

1. NodeDescriptions: Provides the NodeDescriptions class with useful methods to parse a JSON file and create paths.

1. memory_lock and rt_thread: provides useful methods to set up the real-time settings. This functions are taken from (<https://github.com/ros2-realtime-demo/pendulum.git>).

## How to use

This is a library package. Usually, you need to link your binaries (libs or executables) to the library provided by this package. This can be done using the standard ros method (<https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html>).

See also (<https://gitlab.com/volvo_gto/gpss_mvp/control/atr_controller/-/blob/vanilla/CMakeLists.txt>) as an example.

## TODO

Add more utilities to this library.
