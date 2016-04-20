##XBee Protocol Documentation

Quadcopter commands are sent across XBee modules in the form of a string (UTF-8 character array). The quadcopter may respond with a string of its own, in the same format. 

##String Layout
Strings are always in the form of:

    XYABCD     or     XY ABCD

XY are the command bytes, while ABCD are the data (this can represent a 8 to 32 bit integer, depending on the command, but the total string length should never exceed 255 bytes).

Spaces may be added between the command characters and data, but will be stripped before the command is interpreted.

All return values are sent back as data (no command prefix), as integers or bytes depending on length.

##Base Station Commands (to the Quadcopter)
### Global commands
 * `SA` Stop all. This will shut all motors down in case of an emergency stop. No data bytes required.
 * `HB` Heartbeat command. Won't change behavior, but the quadcopter will always respond. No data bytes required.

### Rotation commands
 * `XR #.#` Rotate around the X axis a certain amount, relative to the quadcopter. Data is a 32-bit float.
 * `YR #.#` Rotate around the Y axis a certain amount, relative to the quadcopter. Data is a 32-bit float.
 * `ZR #.#` Rotate around the Z axis a certain amount, relative to the quadcopter. Data is a 32-bit float.
 * `RX` Get X rotation. This is returned as a 32-bit float.
 * `RY` Get Y rotation. This is returned as a 32-bit float.
 * `RZ` Get Z rotation. This is returned as a 32-bit float.

### Motor commands
Note that all of these override position/rotation commands.

 * `M1 #` Set ESC/Motor 1 to a certain value. Data is a 16-bit unsigned integer.
 * `M2 #` Set ESC/Motor 2 to a certain value. Data is a 16-bit unsigned integer.
 * `M3 #` Set ESC/Motor 3 to a certain value. Data is a 16-bit unsigned integer.
 * `M4 #` Set ESC/Motor 4 to a certain value. Data is a 16-bit unsigned integer.
 * `TH #` Set throttle for all four motors. Data is a 16-bit unsigned integer.
 * `AT`   Set Motor control back to automatic (PID) mode.

### PID commands
These commands set constants (Kp, Ki, Kd) for the PID controller, and are applied as soon as they are interpreted. Note that settings these do not set motor control to automatic mode.

 * `KP #.#` Set Kp to a certain value. Data is a 32-bit float.
 * `KI #.#` Set Ki to a certain value. Data is a 32-bit float.
 * `KD #.#` Set Kd to a certain value. Data is a 32-bit float.

Invalid commands will not change behavior.
