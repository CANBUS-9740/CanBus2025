package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSystem;

public class HoldItemInClaw extends Command {
    private final ClawSystem clawSystem;

    public HoldItemInClaw(ClawSystem clawSystem) {
        this.clawSystem = clawSystem;

        addRequirements(clawSystem);
    }

    @Override
    public void initialize() {
        clawSystem.holdItem();
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
       clawSystem.stop();
    }
}
