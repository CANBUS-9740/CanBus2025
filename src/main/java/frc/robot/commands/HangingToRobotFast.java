package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.HangSystem;

public class HangingToRobotFast extends Command {
    private HangSystem hangSystem;

    public HangingToRobotFast(HangSystem hangSystem) {
        this.hangSystem = hangSystem;

        addRequirements(hangSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        hangSystem.toRobotFast();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        hangSystem.stop();
    }
}
