package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;

public class HoldCommand extends Command {
    private final IntakeSystem intakeSystem;

    public HoldCommand(IntakeSystem intakeSystem) {
        this.intakeSystem = intakeSystem;

        addRequirements(intakeSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (intakeSystem.hasItem()) {
            intakeSystem.holdItem();
        } else {
            intakeSystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSystem.stop();
    }
}
