# ROS2-LiteX
An example ROS2 to FPGA Wishbone bridge with usb transport

This implements a simple (trivial) libusb driver for wishbone-over-USB in a ROS2 node.

For the corresponding gateware, see: https://github.com/BrettRD/upgraded-parakeet

## Hack Along

The LiteX build tools emit a file called `build/software/include/generated/csr.h`.
This should be used to set the addresses of the SoC peripherals.
Ideally, this repo should provide `csr_write_simple` and `csr_write_simple`
so that `csr.h` can be included directly.

