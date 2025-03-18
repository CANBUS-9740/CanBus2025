package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.HangSystem;

public class HangingToRobot extends Command {
    private final HangSystem hangSystem;

    public HangingToRobot(HangSystem hangSystem){
        this.hangSystem = hangSystem;

        addRequirements(hangSystem);
    }

    @Override
    public void initialize() {
        hangSystem.moveToPosition(RobotMap.HANGING_ROBOT_ANGLE);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return hangSystem.reachedPosition(RobotMap.HANGING_ROBOT_ANGLE);
    }

    @Override
    public void end(boolean interrupted) {
        hangSystem.stop();
    }
}
