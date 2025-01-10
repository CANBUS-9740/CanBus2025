# REEFSCAPE
## claw 
### sensors
 ???

control with present vbus
### commands 
* intake 
* slow intake
* outtake

## claw axis
### sensors
* absolute encoder
* limit switch (maybe)
* relative encoder (from the motor controller)

control with angle pid
### commands 
* reset (for the relative encoder)
* goToAngle

## arm
### sensors
* relative encoder (from the motor controller)
* limit switch (for when it's close)
* distance sensor (to measure the open size) (maybe)

control by position pid
### commands 
* reset
* setLength

## arm axis
### sendor
* absolute encoder
* limit switches X2 (if it can touch the drivetrain)

control by angle pid
### commands
* reset
* goToAngle

## hanging
### sensors
* magnetic limit switch / encoder

control by present vbus

### commands
* open
* close
