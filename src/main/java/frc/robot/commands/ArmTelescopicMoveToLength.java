package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmJointSystem;
import frc.robot.subsystems.ArmTelescopicSystem;
import frc.robot.subsystems.ClawJointSystem;

public class ArmTelescopicMoveToLength extends Command {
    private final ArmTelescopicSystem armTelescopicSystem;
    private final ArmJointSystem armJointSystem;
    private final ClawJointSystem clawJointSystem;
    private final double targetLength;

    public ArmTelescopicMoveToLength(ArmTelescopicSystem sub, ArmJointSystem armJointSystem, ClawJointSystem clawJointSystem, double targetLength) {
        this.armTelescopicSystem = sub;
        this.targetLength = targetLength;
        this.armJointSystem = armJointSystem;
        this.clawJointSystem = clawJointSystem;

        addRequirements(sub);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
            armTelescopicSystem.moveToLength(targetLength);
    }

    @Override
    public boolean isFinished() {
        return armTelescopicSystem.didReach(targetLength);
    }

    @Override
    public void end(boolean interrupted) {
        armTelescopicSystem.stop();
    }
}
