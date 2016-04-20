# AtlasQuad
A simple quadcopter project build on the DJI F450 quadcopter model.

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


## TODO
* Build a simple Matlab/Python interface to visualize the PID data.
* Add a soft reset command (see https://developer.mbed.org/questions/4680/reset-command-soft-reset/).
* Tune the roll PID.
* Tune the pitch PID.


[1]: [https://launchpad.net/gcc-arm-embedded]
