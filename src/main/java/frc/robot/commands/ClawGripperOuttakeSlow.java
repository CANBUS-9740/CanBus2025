package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawGripperSystem;

public class ClawGripperOuttakeSlow extends Command {
    private final ClawGripperSystem clawGripperSystem;
    private double releaseTime;

    public ClawGripperOuttakeSlow(ClawGripperSystem clawGripperSystem) {
        this.clawGripperSystem = clawGripperSystem;

        addRequirements(clawGripperSystem);
    }

    @Override
    public void initialize() {
        clawGripperSystem.releaseItemSlow();
        releaseTime = 0;
    }

    @Override
    public void execute() {
        if (!clawGripperSystem.hasItem() && releaseTime == 0) {
            releaseTime = Timer.getFPGATimestamp();
        }
    }

    @Override
    public boolean isFinished() {
        return releaseTime + 1 < Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        clawGripperSystem.stop();
    }
}
