# REEFSCAPE

## Robot

### Claw 

The claw is at the end of the arm. It is used to collect corals and algae. It can hold only one of either. It is made up of two sets of two wheels along a triangle operated by a single motor.

To collect an item the wheel (both sides) must touch the items and then be spun as to pull in the item. Then the wheels must continue to spin to keep the item in the claw. To drop the item, the wheels must rotate in the other direction and push it out.

#### Sensors

We must have a way to recognize what item we have collected and whether we collect it or not.
There are several different ways to approach it. Using an optical or mechanical limit switch is probably the easiest. But a distance sensor is also an option.

#### Control

Basic constant speed PercentVBus is likely good enough.

#### Commands 

- `CollectItem`: use a constant speed inward to collect. Activate as part of the collecting flow.
- `KeepItem`: use a slow consant speed inward to keep the item from falling. Activate once an item is collected up until it is released.
- `ReleaseItem`: use a constant speed outward to push it out. Activate as part of a drop or placing flow.

### Claw Joint

The claw as an axis of rotation, allowing to orient the claw. This can be controlled as part of arm orientation.
It is a simple shaft adding a joint.

#### Sensors

We must be able to detect the orientation of the shaft.

- absolute encoder (likely through-bore) placed after the gear box
    - can be connected directly to a motor controller (SparkMax preferrable)
- limit switches (mechanical/optical) to provide a hard limit to the claw motion
    - can be connected directly to a motor controller (SparkMax preferrable) 

#### Control

Use PID loop on the encoder (preferrably from the motor controller) to move the shaft to specific orientations measured in angles.

#### Commands 

- `RotateClawToPosition`: rotate the claw joint to a specific orientation with a PID loop

### Arm

The arm is telescopic with 3 phases of opening. It is an axis, as it can be open to any length, as requested. It is likely operated by a single motor which can push or pull the arm open or close.

#### Sensors

We must be able to measure how much the arm is open, and provide it with a limit of motion.

- Measure Length
    - Relative Encoder from the operating motor to measure how open
    - Distance Sensor on the outside of the arm, measuring distance to a small metal at the edge of the arm
- Limit Switches at the edges of motion
    - Mechanical limit switch to be activated on the arm in closed 

#### Control

Use a position loop PID on the lengtth of the arm. Preferrable from a motor controller.

#### Commands 

- `ExtendArmToLength`: extend or retract arm to a specific length
- `CloseArm`: retract arm until indicated as closed by limit switch

### Arm Joint

The arm is placed on a joint which can move it around ~180 degrees around the robot. Operated by a motor.

#### Sensors

We must be able to detect the orientation of the shaft.

- absolute encoder (likely through-bore) placed after the gear box
    - can be connected directly to a motor controller (SparkMax preferrable)
- limit switches (mechanical/optical) to provide a hard limit to the claw motion
    - can be connected directly to a motor controller (SparkMax preferrable) 

#### Control

Use PID loop on the encoder (preferrably from the motor controller) to move the shaft to specific orientations measured in angles.

#### Commands

- `RotateArmToPosition`: rotate the arm joint to a specific orientation with a PID loop

### Hanging

Hanging uses 2 small claws/hangers that can be opened and closed to catch and hold onto the hanging point. The rest of the work is done by the arm. Each will likely be operated by a motor, either a servo or a DC motor.

#### Sensors

We must be able to tell when and if the claws are open or closed.

- Either limit switches (possibly magnetic) or an encoder from the motor.

#### Control

Basic constant speed PercentVBus is likely good enough.

#### Commands

- `OpenHangers`: open the hanging claws
- `CloseHangers`: close the hanging claws
