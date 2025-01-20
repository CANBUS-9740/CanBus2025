package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.HangingSystem;

public class OpenHanging extends Command {
    private HangingSystem hangingSystem;

    public OpenHanging(HangingSystem hangingSystem) {
        this.hangingSystem = hangingSystem;
    }

    public void initialize() {
        hangingSystem.openLeft();
        hangingSystem.openRight();
    }

    public void execute() {
    }

    public void end(boolean interrupted) {
        hangingSystem.stopLeft();
        hangingSystem.stopRight();
    }

    public boolean isFinished() {
        return hangingSystem.didGetToAngleLeft(RobotMap.HANGING_OPEN_ANGLE) && hangingSystem.didGetToAngleRight(RobotMap.HANGING_OPEN_ANGLE);
    }

}
