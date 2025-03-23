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
    private PWM leds;

    public LedsSystem() {
        leds = new PWM(
                RobotMap.LEDS_PORT
        );
        setColor(0);
    }

    public SequentialCommandGroup showBlinkLights(int first, int second, double time){
        return new SequentialCommandGroup(
                showColor(first),
                Commands.waitSeconds(time),
                showColor(second)
        );
    }

    public Command showColor(int color) {
        return startEnd(()-> setColor(color), ()->{});
    }

    private void setColor(int color) {
        leds.setPulseTimeMicroseconds(color);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Led voltage: ", leds.getPulseTimeMicroseconds());
        SmartDashboard.putNumber("Led pwm: ", leds.getSpeed());

    }
}
