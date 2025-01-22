package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmJointSystem;

import java.util.PrimitiveIterator;

public class ArmJointControlCommand extends Command {
    private final ArmJointSystem sub;
    private double targetPosition;
    private boolean isInTarget;
    private boolean hasNewPosition;
    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State motionProfileGoal;
    private TrapezoidProfile.State motionProfileSetPoint;

    public ArmJointControlCommand(ArmJointSystem sub) {
        this.sub = sub;

        addRequirements(sub);
    }

    @Override
    public void initialize() {
        isInTarget = false;
        hasNewPosition = true;
        targetPosition = 90;
    }

    @Override
    public void execute() {
        if (hasNewPosition) {
           isInTarget = false;

           motionProfile = new TrapezoidProfile(RobotMap.MOTION_PROFILE_CONSTRAINTS);
           motionProfileGoal = new TrapezoidProfile.State(targetPosition, 0);
           motionProfileSetPoint = new TrapezoidProfile.State(sub.getPositionDegrees(), 0);
        }



        if (sub.reachedPosition(targetPosition)) {
            isInTarget = true;
        } else {
            motionProfileSetPoint = motionProfile.calculate(0.02, motionProfileSetPoint, motionProfileGoal);
        }
        sub.moveToPosition(motionProfileSetPoint.position);
    }

    @Override
    public boolean isFinished() {
        return sub.reachedPosition(0) || sub.reachedPosition(180);
    }

    @Override
    public void end(boolean interrupted) {
        sub.stop();
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