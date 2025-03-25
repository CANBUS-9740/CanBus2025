package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.HangSystem;

public class HangingToRobot extends Command {
    private final HangSystem hangSystem;
    private double taregtAngle;

    public HangingToRobot(HangSystem hangSystem, double targetAngle){
        this.hangSystem = hangSystem;
        this.taregtAngle = targetAngle;

        addRequirements(hangSystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        hangSystem.toRobot();
    }

    @Override
    public boolean isFinished() {
        return hangSystem.reachedPosition(taregtAngle);
    }

    @Override
    public void end(boolean interrupted) {
        hangSystem.stop();
    }
}
