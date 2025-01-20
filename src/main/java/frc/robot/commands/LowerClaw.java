package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawJointSubsystem;

public class LowerClaw extends Command {
    private final ClawJointSubsystem clawJointSubsystem;

    public LowerClaw(ClawJointSubsystem clawJointSubsystem){
        this.clawJointSubsystem = clawJointSubsystem;

        addRequirements(clawJointSubsystem);
    }

    public void initialize() {
        clawJointSubsystem.lower();
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
