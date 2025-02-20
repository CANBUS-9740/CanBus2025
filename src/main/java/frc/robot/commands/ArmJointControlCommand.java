package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmJointSystem;

import java.util.PrimitiveIterator;

public class ArmJointControlCommand extends Command {
    private final ArmJointSystem sub;
    private double targetPosition;
    private boolean isInTarget;
    private boolean hasNewPosition;
    private boolean isHolding ;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;

    public ArmJointControlCommand(ArmJointSystem sub) {
        this.sub = sub;

        addRequirements(sub);
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

            SmartDashboard.putNumber("ArmJointCommandTarget", targetPosition);
            SmartDashboard.putBoolean("ArmJointCommandHolding", isHolding);
            SmartDashboard.putBoolean("ArmJointCommandInTarget", false);

           if(isHolding) {
               motionProfile = new TrapezoidProfile(RobotMap.MOTION_PROFILE_CONSTRAINTS);
               motionProfileGoal = new TrapezoidProfile.State(targetPosition, 0);
               motionProfileSetPoint = new TrapezoidProfile.State(sub.getPositionDegrees(), 0);
           } else {
               SmartDashboard.putBoolean("ArmJointCommandInTarget", true);
               isInTarget = true;
               sub.stop();
           }
        }

        if (!isHolding) {
            return;
        }

        if (!isInTarget && sub.reachedPosition(targetPosition)) {
            SmartDashboard.putBoolean("ArmJointCommandInTarget", true);
            isInTarget = true;
        }

        if (isInTarget) {
            if (targetPosition <= RobotMap.ARM_JOINT_MINIMUM_ANGLE || targetPosition >= RobotMap.ARM_JOINT_MAXIMUM_ANGLE) {
                stopHolding();
            } else {
                sub.moveToPosition(targetPosition);
            }
        } else {
            motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);
            sub.moveToPosition(motionProfileSetPoint.position);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        sub.stop();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    public void setTargetPosition(double newPosition){
        targetPosition = newPosition;
        hasNewPosition = true;
        isHolding = true;
    }

    public void stopHolding(){
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