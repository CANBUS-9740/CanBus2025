package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmJointSystem;

public class MoveArmJointToPosition extends Command {
    private final ArmJointSystem armJointSystem;
    private final double targetPositionDegrees;

    private MoveArmJointToPosition(ArmJointSystem armJointSystem, double targetPositionDegrees) {
        this.armJointSystem = armJointSystem;
        this.targetPositionDegrees = targetPositionDegrees;

        addRequirements(armJointSystem);
    }

    @Override
    public void initialize() {
        armJointSystem.moveToPosition(targetPositionDegrees);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armJointSystem.stop();
    }
}
