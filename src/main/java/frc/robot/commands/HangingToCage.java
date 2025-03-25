package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.HangSystem;

public class HangingToCage extends Command {
    private final HangSystem hangSystem;

    public HangingToCage(HangSystem hangSystem){
        this.hangSystem = hangSystem;

        addRequirements(hangSystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        hangSystem.toCage();
    }

    @Override
    public boolean isFinished() {
        return hangSystem.reachedPosition(RobotMap.HANGING_CAGE_ANGLE);
    }

    @Override
    public void end(boolean interrupted) {
        hangSystem.stop();
    }
}
