package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmJointSubsystem;

public class ArmJointHold extends Command {
    private final ArmJointSubsystem armJointSubsystem;

    private ArmJointHold(ArmJointSubsystem armJointSubsystem){
        this.armJointSubsystem = armJointSubsystem;

        addRequirements(armJointSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armJointSubsystem.hold();
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
