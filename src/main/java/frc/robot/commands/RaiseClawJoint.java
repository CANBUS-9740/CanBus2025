package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawJointSystem;

public class RaiseClawJoint extends Command {
    private final ClawJointSystem clawJointSystem;

    public RaiseClawJoint(ClawJointSystem clawJointSystem){
        this.clawJointSystem = clawJointSystem;

        addRequirements(clawJointSystem);
    }
    public void initialize() {
        clawJointSystem.raise();
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
