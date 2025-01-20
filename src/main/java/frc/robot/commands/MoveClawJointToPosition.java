package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawJointSystem;

public class MoveClawJointToPosition extends Command {
    private final ClawJointSystem clawJointSystem;
    private final double positionDegrees;

    public MoveClawJointToPosition(ClawJointSystem clawJointSystem, double positionDegrees){
        this.clawJointSystem = clawJointSystem;
        this.positionDegrees = positionDegrees;

        addRequirements(clawJointSystem);
    }
    public void initialize() {
        clawJointSystem.moveToPosition(positionDegrees);
    }

    public void execute() {
    }

    public void end(boolean interrupted) {
        clawJointSystem.stop();
    }

    public boolean isFinished() {
        return false;
    }
}
