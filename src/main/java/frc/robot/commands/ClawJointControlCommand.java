package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmJointSystem;
import frc.robot.subsystems.ClawJointSystem;

public class ClawJointControlCommand extends Command {
    private final ArmJointSystem armJointSystem;
    private final ClawJointSystem clawJointSystem;
    private double targetPosition;
    private boolean isInTarget;
    private boolean hasNewPosition;
    private boolean isHolding ;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;

    public ClawJointControlCommand(ArmJointSystem armJointSystem, ClawJointSystem clawJointSystem) {
        this.armJointSystem = armJointSystem;
        this.clawJointSystem = clawJointSystem;

        addRequirements(clawJointSystem);
    }

    @Override
    public void initialize() {
        isHolding = false;
        isInTarget = false;
        hasNewPosition = true;
    }

    @Override
    public void execute() {
        if (DriverStation.isDisabled() && isHolding) {
            stopHolding();
        }

        if (hasNewPosition) {
            isInTarget = false;
            hasNewPosition = false;

            SmartDashboard.putNumber("ClawJointCommandTarget", targetPosition);
            SmartDashboard.putBoolean("ClawJointCommandHolding", isHolding);
            SmartDashboard.putBoolean("ClawJointCommandInTarget", false);

            if(isHolding) {
                motionProfile = new TrapezoidProfile(RobotMap.CLAWJOINT_MOTION_PROFILE_CONSTRAINTS);
                motionProfileGoal = new TrapezoidProfile.State(targetPosition, 0);
                motionProfileSetPoint = new TrapezoidProfile.State(clawJointSystem.getRawPositionDegrees(), 0);
            } else {
                SmartDashboard.putBoolean("ClawJointCommandInTarget", true);
                isInTarget = true;
                clawJointSystem.stop();
            }
        }

        if (!isHolding) {
            return;
        }

        if (!isInTarget && clawJointSystem.didReachPosition(targetPosition)) {
            SmartDashboard.putBoolean("ClawJointCommandInTarget", true);
            isInTarget = true;
        }

        if (isInTarget) {
            clawJointSystem.moveToPosition(targetPosition, armJointSystem.getLogicalPositionDegrees());
        } else {
            motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);
            clawJointSystem.moveToPosition(motionProfileSetPoint.position, armJointSystem.getLogicalPositionDegrees());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        clawJointSystem.stop();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    public double getTargetPosition() {
        if (isHolding) {
            return targetPosition;
        }

        return -1;
    }

    public void setTargetPosition(double newPosition) {
        if (newPosition < RobotMap.CLAWJOINT_MINIMUM_ANGLE || newPosition > RobotMap.CLAWJOINT_MAXIMUM_ANGLE) {
            return;
        }

        targetPosition = newPosition;
        hasNewPosition = true;
        isHolding = true;
    }

    public void stopHolding() {
        isHolding = false;
        hasNewPosition = true;
    }

    public boolean isAtTargetPosition() {
        if (hasNewPosition) {
            return false;
        } else {
            return isInTarget;
        }
    }
}