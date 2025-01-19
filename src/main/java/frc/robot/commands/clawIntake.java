package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSystem;

public class clawIntake extends Command {

    private final ClawSystem clawSystem;

    public clawIntake(ClawSystem clawSystem) {
        this.clawSystem = clawSystem;

        addRequirements(clawSystem);
    }

    @Override
    public void initialize() {
        clawSystem.collectItem();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return clawSystem.hasItem();
    }

    @Override
    public void end(boolean interrupted) {
        clawSystem.stop();
    }
}
