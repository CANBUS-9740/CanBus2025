package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmJointSubsystem;

public class MoveArmToPosition extends Command {
    private final ArmJointSubsystem armJointSubsystem;

    private MoveArmToPosition(ArmJointSubsystem armJointSubsystem){
        this.armJointSubsystem = armJointSubsystem;

        addRequirements(armJointSubsystem);
    }

    public void initialize() {
    }

    public void execute(double targetPosition) {
        armJointSubsystem.moveToPosition(targetPosition);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        armJointSubsystem.stop();
    }
}
