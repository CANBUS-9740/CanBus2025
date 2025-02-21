package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmTelescopicSystem;

public class ArmTelescopicReset extends Command {
    private final ArmTelescopicSystem sub;

    private final Timer waitTimer;

    public ArmTelescopicReset(ArmTelescopicSystem sub) {
        this.sub = sub;
        waitTimer = new Timer();

        addRequirements(sub);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (sub.getResetLimitSwitch()) {
            if (!waitTimer.isRunning()) {
                sub.stop();
                waitTimer.restart();
            }
        } else {
            sub.retract();
        }
    }

    @Override
    public boolean isFinished() {
        return waitTimer.isRunning() && waitTimer.hasElapsed(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            sub.setEncoderValue(0);
        }

        waitTimer.stop();
        sub.stop();
    }
}
