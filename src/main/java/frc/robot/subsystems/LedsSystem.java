package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LedsSystem extends SubsystemBase {
    private Spark leds;

    public LedsSystem() {
        leds = new Spark(RobotMap.LEDS_PORT);
    }

    public void setColor(double color) {
        leds.set(color);
    }

    @Override
    public void periodic() {

    }
}
