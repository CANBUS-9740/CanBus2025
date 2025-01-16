package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawJointSubsystem;

public class HoldClaw extends Command {
    private final ClawJointSubsystem clawJointSubsystem;

    public HoldClaw(ClawJointSubsystem clawJointSubsystem){
        this.clawJointSubsystem = clawJointSubsystem;

        addRequirements(clawJointSubsystem);
    }

    public void initialize() {
        clawJointSubsystem.hold();
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
