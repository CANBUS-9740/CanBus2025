package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmTelescopicSystem;

public class ArmTelescopicHold extends Command {
    private final ArmTelescopicSystem sub;

    public ArmTelescopicHold(ArmTelescopicSystem sub) {
        this.sub = sub;

        addRequirements(sub);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        sub.hold();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        sub.stop();
    }
}
