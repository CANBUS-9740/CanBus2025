package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawJointSubsystem;

public class MoveClawToPosition extends Command {
    private final ClawJointSubsystem clawJointSubsystem;
    private final double positionDegrees;

    public MoveClawToPosition(ClawJointSubsystem clawJointSubsystem, double positionDegrees){
        this.clawJointSubsystem = clawJointSubsystem;
        this.positionDegrees = positionDegrees;

        addRequirements(clawJointSubsystem);
    }
    public void initialize() {
        clawJointSubsystem.moveToPosition(positionDegrees);
    }

    public void execute() {
    }

    public void end(boolean interrupted) {
        clawJointSubsystem.stop();
    }

    public boolean isFinished() {
        return false;
    }
}
