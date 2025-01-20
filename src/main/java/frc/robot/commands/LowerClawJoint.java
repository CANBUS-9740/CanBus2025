package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawJointSystem;

public class LowerClawJoint extends Command {
    private final ClawJointSystem clawJointSystem;

    public LowerClawJoint(ClawJointSystem clawJointSystem){
        this.clawJointSystem = clawJointSystem;

        addRequirements(clawJointSystem);
    }

    public void initialize() {
        clawJointSystem.lower();
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
