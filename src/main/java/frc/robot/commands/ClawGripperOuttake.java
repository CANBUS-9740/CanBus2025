package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawGripperSystem;

public class ClawGripperOuttake extends Command {
    private final ClawGripperSystem clawGripperSystem;

    public ClawGripperOuttake(ClawGripperSystem clawGripperSystem) {
        this.clawGripperSystem = clawGripperSystem;

        addRequirements(clawGripperSystem);
    }

    @Override
    public void initialize() {
        clawGripperSystem.releaseItem();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;//!clawGripperSystem.hasItem();
    }

    @Override
    public void end(boolean interrupted) {
        clawGripperSystem.stop();
    }
}
