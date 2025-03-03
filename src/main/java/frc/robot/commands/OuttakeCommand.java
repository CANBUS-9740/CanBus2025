package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;

public class OuttakeCommand extends Command {
    private final IntakeSystem intakeSystem;

    public OuttakeCommand(IntakeSystem intakeSystem) {
        this.intakeSystem = intakeSystem;

        addRequirements(intakeSystem);
    }

    @Override
    public void initialize() {
        intakeSystem.releaseItem();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return !intakeSystem.hasItem();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSystem.stop();
    }
}
