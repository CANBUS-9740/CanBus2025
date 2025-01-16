package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawJointSubsystem;

public class RaiseClaw extends Command {
    private final ClawJointSubsystem clawJointSubsystem;


    public RaiseClaw(ClawJointSubsystem clawJointSubsystem){
        this.clawJointSubsystem = clawJointSubsystem;

        addRequirements(clawJointSubsystem);
    }
    public void initialize() {
        clawJointSubsystem.raise();
    }

    public void execute() {

    }

    public void end(boolean interrupted) {
        clawJointSubsystem.stop();
    }

    public boolean isFinished() {
        return false;
    }

}
