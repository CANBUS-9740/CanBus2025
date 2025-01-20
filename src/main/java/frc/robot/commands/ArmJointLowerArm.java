package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmJointSubsystem;

public class ArmJointLowerArm extends Command {
    private final ArmJointSubsystem armJointSubsystem;

    public ArmJointLowerArm(ArmJointSubsystem armJointSubsystem){
        this.armJointSubsystem = armJointSubsystem;

        addRequirements(armJointSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armJointSubsystem.lower();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armJointSubsystem.stop();
    }
}
