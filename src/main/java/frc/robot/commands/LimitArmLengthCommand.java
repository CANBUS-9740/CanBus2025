package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.ArmJointSystem;
import frc.robot.subsystems.ArmTelescopicSystem;

public class LimitArmLengthCommand extends Command {
    private ArmTelescopicSystem armTelescopicSystem;
    private ArmJointSystem armJointSystem;

    private double armLength;
    private double armAngle;
    private double distance;
    private double clawAngle;

    public LimitArmLengthCommand(ArmTelescopicSystem armTelescopicSystem, ArmJointSystem armJointSystem, double armLength, double armAngle, double distance, double clawAngle ){
       this.armJointSystem = armJointSystem;
       this.armTelescopicSystem = armTelescopicSystem;
        this.armLength = armLength;
       this.armAngle = armAngle;
       this.distance = distance;
       this.clawAngle = clawAngle;

       addRequirements(armTelescopicSystem);
        addRequirements(armJointSystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double currentAngle = armJointSystem.getPositionDegrees();
        double currentLength = armTelescopicSystem.getLengthMeters();

        boolean legal = !armTelescopicSystem.isCommandIsValid(currentLength,currentAngle,distance,clawAngle);

        if(legal){
            armTelescopicSystem.moveToLength(armLength);
        } else{
            armTelescopicSystem.stop();
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armTelescopicSystem.stop();
    }
}
