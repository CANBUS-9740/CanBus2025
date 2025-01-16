# REEFSCAPE

## Knowledge Completion

### SparkMax

REV has modified the API for spark max pretty significantly. See [here](https://github.com/tomtzook/frc-learn-docs/blob/master/devices/rev/spark-max.md) for more complete info and [here](https://github.com/REVrobotics/REVLib-Examples/tree/main/Java/SPARK) for examples.

The following shows initializing a SparkMax and reseting its setting
```java
public class Robot extends TimedRobot {

    private SparkMax motor;

    @Override
    public void robotInit() {
        motor = new SparkMax(RobotMap.MOTOR_IDENTIFIER, SparkLowLevel.MotorType.kBrushless);
    
        SparkMaxConfig config = new SparkMaxConfig();
        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
  }
}
```

## Robot

### Claw 

The claw is at the end of the arm. It is used to collect corals and algae. It can hold only one of either. It is made up of two sets of two wheels along a triangle operated by a single motor.

To collect an item the wheel (both sides) must touch the items and then be spun as to pull in the item. Then the wheels must continue to spin to keep the item in the claw. To drop the item, the wheels must rotate in the other direction and push it out.

#### Sensors

We must have a way to recognize what item we have collected and whether we collect it or not.
There are several different ways to approach it. Using an optical or mechanical limit switch is probably the easiest. But a distance sensor is also an option.

Discussed about placing an IR sensor cutting through the center to detect the object.

#### Control

Basic constant speed PercentVBus is likely good enough.

#### Requirements

- Detecting that an object is being held
- Detecting which object is being held (optional)
- Easy and quick collect of object (both Coral and Algae)
- Easy and quick relase of object (both Coral and Algae)
- Collected object is held in place

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

#### Requirements

- The joint is held in place when wanted
- Easy, fluid and accurate motion to different orientations
- Soft limits
- Hard limits
- We'll likely want to use similar arm control scheme as last year (with highly capable arm-command)

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

#### Requirements

- Fluid and accurate control of extension
- Accurate tracking of extension
- Soft limits
- Hard limits
- Limitation of arm extension per position of joint to keep within rules

### Arm Joint

The arm is placed on a joint which can move it around ~180 degrees around the robot. Operated by a motor.

#### Sensors

We must be able to detect the orientation of the shaft.

- absolute encoder (likely through-bore) placed after the gear box
    - can be connected directly to a motor controller (SparkMax preferrable)
- limit switches (mechanical/optical) to provide a hard limit to the claw motion
    - can be connected directly to a motor controller (SparkMax preferrable) 

#### Control

Use PID loop on the encoder (preferrably from the motor controller) to move the shaft to specific orientations measured in angles. Also Possible to implement motion profiling on the RoboRIO instead

#### Requirements

- The joint is held in place when wanted
- Easy, fluid and accurate motion to different orientations
- Soft limits
- Hard limits
- Possibly motion profiling
- We'll likely want to use similar arm control scheme as last year (with highly capable arm-command)

### Hanging

Hanging uses 2 small claws/hangers that can be opened and closed to catch and hold onto the hanging point. The rest of the work is done by the arm. Each will likely be operated by a motor, either a servo or a DC motor.

Likely a servo with a mechanical locking latch to prevent it from opening.

#### Sensors

We must be able to tell when and if the claws are open or closed.

Either limit switches (possibly magnetic) or an encoder from the motor for DC Motor.
For Servo we can just rely on the servo positioning.

#### Control

Basic constant speed PercentVBus is likely good enough.

#### Requirements

- Locks in place
- Good positional opening
- Good positional closing

## Implementation

### Phase 1

In this phase we'll be implementing the basic subsystems of the robot. This is the base of the robot from which we can do the rest. For each subsystem, please read the description and specs of the subsystems to understand how it works.

#### ClawSystem

`ClawSystem` operates only the collecting part of the claw, that is a single NEO 1.1 motor with SparkMax operating 2 sets of 2 wheels. This is in addition to the limit switch indicating if an item is held.

- Create the subsystem
- Create the motor controller
- Create the limit switch
- Implement `public boolean hasItem()` method
    - returns `true` if the limit switch indicates the claw holds an item, `false` otherwise
    - query the limit switch with `.get`
    - remember that the limit switch is normally open and so `get` returns `true` when not pressed
- Implement `public void rotateCollect()` method
    - rotate the motor at a constant speed for collecting an item
    - use the speed value `0.8`
- Implement `public void rotateRelease()` method
    - rotate the motor at a constant speed for release an item
    - use the speed value `-0.5`
- Implement `public void rotateHold()` method
    - rotate the motor at a constant speed for holding a collected item in place 
    - use the speed value `0.2`
- Implement `public void stop()` method
    - stops the motor
- Add `periodic` method which prints the values from
    - `hasItem`

Implement Basic Operational Commands for the subsystem:
- `CollectItemWithClaw`: use `rotateCollect` to collect an item. end when an item is collected
- `ReleaseItemWithClaw` use `rotateRelease` to release an item. end when an item is released
- `HoldItemInClaw`: use `rotateHold` to hold the item in place. never ends
 
#### ClawJointSystem

The `ClawJointSystem` operates the joint of the claw. This joint allows orienting the claw up and down up to certain limits. It is operated by a single NEO 1.1 motor with a SparkMax, has 2 limit switches that are connected to the SparkMax and provide hard-limits to the motion and a _Through-Bore_ encoder connected to the SparkMax as an _Absolute Encoder_. The _Through-Bore_ will be used as the encoder for tracking the joint position and is placed after any gear boxes. 

For Through-Bore in absolute mode, start pulse is `1us`, while end pulse is `1024us`.

- Create the subsystem
- Create the motor controller (see the SparkMax doc attached at the start of the README for how)
    - configure encoder 
    - configure PID (currently all gains are 0)
    - configure soft-limits
    - configure hard-limits
- Implement `public double getPositionDegrees()` method
    - return the position of the joint as per the encoder connected to SparkMax
- Implement `public void moveToPosition(double positionDegrees)` method
    - starts a position closed-loop on the SparkMax which moves the joint to the requested position 
- Implement `public void move(double speed)` method
    - rotate the motor at the requested speed
    - speed is a _PercentVBus_ value
- Implement `public void raise()` method
    - rotate the motor at a constant speed for raising the joint
    - use the speed value `0.8`
- Implement `public void lower()` method
    - rotate the motor at a constant speed for lowering the joint
    - use the speed value `-0.5`
- Implement `public void hold()` method
    - rotate the motor at a constant speed for holding it in place
    - use the speed value `0.2`
- Implement `public void stop()` method
    - stops the motor
- Add `periodic` method which prints the values from
    - `getPositionDegrees`
    - the hard limits from SparkMax (whether they are enabled)

Implement Basic Operational Commands for the subsystem:
- `RaiseClaw`: use `raise` to raise the claw while the command is running. command has no stop reason
- `LowerClaw`: use `lower` to lower the claw while the command is running. command has no stop reason
- `HoldClaw`: use `hold` to keep the claw in place while the command is running. command has no stop reason
- `MoveClawToPosition`: receive target position and orient the claw joint to reach the requested position. once in position, keep the claw in place. command has no stop reason

 #### ArmTelescopicSystem

The `ArmTelescopicSystem` operates the extension of the telescopic part of the arm. This allows the arm to extend and retract itself. The telescopic arm is made up of 3 phases which - one main phase and 2 phases which can extend out of it. By default the arm is fully extended by a set of springs pushing the phases open. To retract the arm one must use the motor which pulls the phases in. This is done by a rope attached to the end of the third phase. When the motor rotates it pulls the rope in around a drum retracting the arm. When the motor releases the rope the arm extends back again. This is a similar principle to an elevator with the springs being like the gravity.

The motor is a single NEO 1.1 connected to a SparkMax. The integrated encoder of the arm can be used to track how much the arm is extended. The conversion between motor rotation to arm length includes the gear ratio, and the radius of the drum over which the rope wraps around. Should be `lengthMeters = motorRotations / gearRatio * drumCircumference`. 

PID can be executed over the SparkMax.

- Create the subsystem
- Create the motor controller (see the SparkMax doc attached at the start of the README for how)
    - configure encoder (use conversion factors)
    - configure PID (currently all gains are 0)
    - configure soft-limits
- Implement `public double getLengthMeters()` method
    - return the length of the arm in meters per the encoder
- Implement `public void moveToLength(double lengthMeters)` method
    - starts a position closed-loop on the SparkMax which moves the arm to the requested position 
- Implement `public void move(double speed)` method
    - rotate the motor at the requested speed
    - speed is a _PercentVBus_ value
- Implement `public void extend()` method
    - rotate the motor at a constant speed for extending the arm
    - use the speed value `-0.2`
- Implement `public void retract()` method
    - rotate the motor at a constant speed for retracting the arm
    - use the speed value `0.5`
- Implement `public void hold()` method
    - rotate the motor at a constant speed for holding it in place
    - use the speed value `0.2`
- Implement `public void stop()` method
    - stops the motor
- Add `periodic` method which prints the values from
    - `getLengthMeters`

 #### ArmJointSystem

The `ArmJointSystem` operates the joint of the arm. This joint allows orienting the arm up and down up to certain limits. It is operated by a single NEO 1.1 motor with a SparkMax, has 2 limit switches that are connected to the SparkMax and provide hard-limits to the motion and a _Through-Bore_ encoder connected to the SparkMax as an _Absolute Encoder_. The _Through-Bore_ will be used as the encoder for tracking the joint position and is placed after any gear boxes. 

For Through-Bore in absolute mode, start pulse is `1us`, while end pulse is `1024us`.

- Create the subsystem
- Create the motor controller (see the SparkMax doc attached at the start of the README for how)
    - configure encoder 
    - configure PID (currently all gains are 0)
    - configure soft-limits
    - configure hard-limits
- Implement `public double getPositionDegrees()` method
    - return the position of the joint as per the encoder connected to SparkMax
- Implement `public void moveToPosition(double positionDegrees)` method
    - starts a position closed-loop on the SparkMax which moves the joint to the requested position 
- Implement `public void move(double speed)` method
    - rotate the motor at the requested speed
    - speed is a _PercentVBus_ value
- Implement `public void raise()` method
    - rotate the motor at a constant speed for raising the joint
    - use the speed value `0.8`
- Implement `public void lower()` method
    - rotate the motor at a constant speed for lowering the joint
    - use the speed value `-0.5`
- Implement `public void hold()` method
    - rotate the motor at a constant speed for holding it in place
    - use the speed value `0.2`
- Implement `public void stop()` method
    - stops the motor
- Add `periodic` method which prints the values from
    - `getPositionDegrees`
    - the hard limits from SparkMax (whether they are enabled)

Implement Basic Operational Commands for the subsystem:
- `RaiseArm`: use `raise` to raise the arm while the command is running. command has no stop reason
- `LowerArm`: use `lower` to lower the arm while the command is running. command has no stop reason
- `HoldArm`: use `hold` to keep the arm in place while the command is running. command has no stop reason
- `MoveArmToPosition`: receive target position and orient the arm joint to reach the requested position. once in position, keep the arm in place. command has no stop reason

 #### HangingSystem

The `HangingSystem` is used to hook the robot's arm to the cages of the Barge. There are two clips on the lower part of the arm, next to each other. To hang, these hooks must be opened, then driven into the cage and closed to hang on to the cage. A mechanical look is present which will allow the hooks to keep a hold on the cage. Two Servos drive the hooks (1 each). These are not powerful, but can easily move the hooks open and close.

The _Servo_s are basic low power motor with integrated feedback control allowing them to move accurately. They are controlled via PWM. _Servo_'s are not use for continous rotational motion, but rather limited positional change. Read further [here](https://www.sparkfun.com/servos).

To control a _Servo_, use the `Servo` class.
```java
public class Robot extends TimedRobot {

    private Servo servo;

    @Override
    public void robotInit() {
        servo = new Servo(RobotMap.SERVO_PORT);
    }

    @Override
    public void teleopInit() {
        servo.setAngle(45); // rotate the servo to 45 degrees.
    }
}
```

- Create the subsystem
- Create the 2 servos
- Implement `public void open()` method
    - rotate both servos to position `0`
- Implement `public void close()` method
    - rotate both servos to position `180`
 
> [!NOTE]
> There is much that is still unclear about how the hooks will work and how the servos will be used.
> When more is known, we will change
