package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.HangingSystem;

public class CloseHanging extends Command {
    private HangingSystem hangingSystem;

    public CloseHanging(HangingSystem hangingSystem) {
        this.hangingSystem = hangingSystem;
    }

    public void initialize() {
        hangingSystem.closeLeft();
        hangingSystem.closeRight();
    }

    public void execute() {
    }

    public void end(boolean interrupted) {
        hangingSystem.stopLeft();
        hangingSystem.stopRight();
    }

    public boolean isFinished() {
        return hangingSystem.didGetToAngleLeft(RobotMap.HANGING_CLOSE_ANGLE) && hangingSystem.didGetToAngleRight(RobotMap.HANGING_CLOSE_ANGLE);
    }

}
