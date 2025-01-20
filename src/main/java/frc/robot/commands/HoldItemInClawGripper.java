package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawGripperSystem;

public class HoldItemInClawGripper extends Command {
    private final ClawGripperSystem clawGripperSystem;

    public HoldItemInClawGripper(ClawGripperSystem clawGripperSystem) {
        this.clawGripperSystem = clawGripperSystem;

        addRequirements(clawGripperSystem);
    }

    @Override
    public void initialize() {
        clawGripperSystem.holdItem();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
       clawGripperSystem.stop();
    }
}
