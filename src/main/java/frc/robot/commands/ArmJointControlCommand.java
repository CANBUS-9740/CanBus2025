package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmJointSystem;
import frc.robot.subsystems.ArmTelescopicSystem;
import frc.robot.subsystems.ClawJointSystem;

public class ArmJointControlCommand extends Command {
    private final ArmJointSystem armJointSystem;
    private final ArmTelescopicSystem armTelescopicSystem;
    private final ClawJointSystem clawJointSystem;
    private double targetPosition;
    private boolean isInTarget;
    private boolean hasNewPosition;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;

    public ArmJointControlCommand(ArmJointSystem sub, ArmTelescopicSystem armTelescopicSystem, ClawJointSystem clawJointSystem) {
        this.armJointSystem = sub;
        this.armTelescopicSystem = armTelescopicSystem;
        this.clawJointSystem = clawJointSystem;

        addRequirements(sub);
    }

    @Override
    public void initialize() {
        isInTarget = false;
        hasNewPosition = true;
        targetPosition = RobotMap.ARM_JOINT_DEFAULT_POSITION;
    }

    @Override
    public void execute() {
        if (hasNewPosition) {
           isInTarget = false;
           hasNewPosition = false;

           motionProfile = new TrapezoidProfile(RobotMap.MOTION_PROFILE_CONSTRAINTS);
           motionProfileGoal = new TrapezoidProfile.State(targetPosition, 0);
           motionProfileSetPoint = new TrapezoidProfile.State(armJointSystem.getPositionDegrees(), 0);
        }

        if (!isInTarget && armJointSystem.reachedPosition(targetPosition)) {
            isInTarget = true;
        }

        if (isInTarget) {
            if (targetPosition == 0 || targetPosition == 180) {
                armJointSystem.stop();
            } else {
                armJointSystem.moveToPosition(targetPosition);
            }
        } else {
            motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);
            armJointSystem.moveToPosition(motionProfileSetPoint.position);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armJointSystem.stop();
    }

    public void setTargetPosition(double newPosition){
        targetPosition = newPosition;
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