package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawGripperSystem;

public class ClawGripperIntake extends Command {

    private final ClawGripperSystem clawGripperSystem;


    public ClawGripperIntake(ClawGripperSystem clawGripperSystem) {
        this.clawGripperSystem = clawGripperSystem;

        addRequirements(clawGripperSystem);
    }

    @Override
    public void initialize() {
        clawGripperSystem.collectItem();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return clawGripperSystem.hasItem();
    }

    @Override
    public void end(boolean interrupted) {
        clawGripperSystem.stop();
    }
}
