package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSystem;

public class clawOuttake extends Command {
    private final ClawSystem clawSystem;

    public clawOuttake(ClawSystem clawSystem) {
        this.clawSystem = clawSystem;

        addRequirements(clawSystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        clawSystem.releaseItem();
    }

    @Override
    public boolean isFinished() {
        return !clawSystem.hasItem();
    }

    @Override
    public void end(boolean interrupted) {
        clawSystem.stop();
    }
}
