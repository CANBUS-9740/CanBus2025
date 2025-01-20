package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmJointSystem;

public class ArmJointHold extends Command {
    private final ArmJointSystem armJointSystem;

    private ArmJointHold(ArmJointSystem armJointSystem){
        this.armJointSystem = armJointSystem;

        addRequirements(armJointSystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armJointSystem.hold();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armJointSystem.stop();
    }
}
