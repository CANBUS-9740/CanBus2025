package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ClawGripperSystem;

public class ClawGripperOuttake extends Command {
    private final ClawGripperSystem clawGripperSystem;
    private final edu.wpi.first.wpilibj.Timer timer;

    public
    ClawGripperOuttake(ClawGripperSystem clawGripperSystem) {
        this.clawGripperSystem = clawGripperSystem;
        timer = new Timer();

        addRequirements(clawGripperSystem);
    }

    @Override
    public void initialize() {
        clawGripperSystem.releaseItem();
        timer.start();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return !clawGripperSystem.hasItem() && timer.get() > 0.5;
    }

    @Override
    public void end(boolean interrupted) {
        clawGripperSystem.stop();
        timer.reset();
    }
}
