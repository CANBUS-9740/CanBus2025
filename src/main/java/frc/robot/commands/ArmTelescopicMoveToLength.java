package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmTelescopicSystem;

public class ArmTelescopicMoveToLength extends Command {
    private final ArmTelescopicSystem sub;
    private final double targetLength;

    public ArmTelescopicMoveToLength(ArmTelescopicSystem sub, double targetLength) {
        this.sub = sub;
        this.targetLength = targetLength;

        addRequirements(sub);
    }

    @Override
    public void initialize() {
        sub.moveToLength(targetLength);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return sub.didReach(targetLength);
    }

    @Override
    public void end(boolean interrupted) {
        sub.stop();
    }
}
