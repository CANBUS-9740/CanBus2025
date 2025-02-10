package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmJointSystem;
import frc.robot.subsystems.ArmTelescopicSystem;

public class ArmTelescopicMoveToLength extends Command {
    private final ArmTelescopicSystem armTelescopicSystem;
    private final ArmJointSystem armJointSystem;
    private final double targetLength;

    private final double distance;
    private final double clawAngle;

    public ArmTelescopicMoveToLength(ArmTelescopicSystem sub,ArmJointSystem armJointSystem , double targetLength, double distance, double clawAngle) {
        this.armTelescopicSystem = sub;
        this.targetLength = targetLength;
        this.armJointSystem = armJointSystem;
        this.distance = distance;
        this.clawAngle = clawAngle;

        addRequirements(sub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if((Robot.isCommandIsValid(armTelescopicSystem.getLengthMeters(), armJointSystem.getPositionDegrees(), distance, clawAngle) && armTelescopicSystem.getMotorSpeed() > 0)) {
            armTelescopicSystem.hold();
        } else {
            armTelescopicSystem.moveToLength(targetLength);
        }
    }

    @Override
    public boolean isFinished() {
        return armTelescopicSystem.didReach(targetLength);
    }

    @Override
    public void end(boolean interrupted) {
        armTelescopicSystem.stop();
    }
}
