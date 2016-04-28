# AtlasQuad
A simple quadcopter project built on the DJI F450 quadcopter model.

## Building
To build the project without debugging output, simply issue the command:

```
make
```

For debugging output, build with the command:

```
make DEBUG=1
```

## Requirements

* [GCC ARM Embedded toolchain][1]
* [Optional] Simulink w/ SimElectronics, SimMechanics, and Simscape


# Directory Layout

* `Documents` - Contains various datasheets and PDFs of the Simulink model of the AtlasQuad system.
* `Simulink` - Contains the Simulink model of the AtlasQuad system.
* `src` - Contains all of the source code for the AtlasQuad flight controller.
  * `main.cpp` - The main flight controller application.
  * `config.h` - Contains system configurations.
  * `xbeeuart.cpp/h` - Contains code used to communicate to the XBee module onboard the AtlasQuad system.
  * `ADXL345/` - Contains the library files developed to communicate to the ADXL345 accelerometer.
  * `ITG3200/` - Contains the library files developed to communicate to the ITG3200 gyro.
  * `mbed/` - Contains the sources for the Mbed platform.
  * `motors/` - Contains source for motor control through the ESCs.
  * `PID/` - Contains source for the PID controllers.
* `Utilities` - Contains two utilities that we developed to help tune the PIDs and to command the quadcopter from a remote base station.
* `XCTU` - Contains an XML file with XBee packet configurations to communicate to the AtlasQuad system through the XCTU software.

## TODO

[1]: [https://launchpad.net/gcc-arm-embedded]
