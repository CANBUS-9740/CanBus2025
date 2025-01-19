package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmTelescopicSystem;

public class ArmTelescopicReset extends Command {
    private final ArmTelescopicSystem sub;

    public ArmTelescopicReset(ArmTelescopicSystem sub) {
        this.sub = sub;

        addRequirements(sub);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        sub.moveToLength(RobotMap.ARM_TELESCOPIC_RESET_LENGTH_METERS);
    }

    @Override
    public boolean isFinished() {
        return sub.ArmResearchLength(RobotMap.ARM_TELESCOPIC_RESET_LENGTH_METERS) && sub.getResetLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        sub.setEncoderValue(0);
        sub.stop();
    }
}
