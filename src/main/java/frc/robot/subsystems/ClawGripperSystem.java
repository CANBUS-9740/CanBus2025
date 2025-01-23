package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClawGripperSystem extends SubsystemBase {
    private final SparkMax motor;
    private final DigitalInput sensor;


    public ClawGripperSystem() {
        sensor = new DigitalInput(RobotMap.CLAW_SWITCH_PORT);
        motor = new SparkMax(RobotMap.CLAW_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    }

    public void collectItem() {
        motor.set(0.8);
    }

    public void releaseItem(){
        motor.set(-0.5);
    }
    public void holdItem(){
        motor.set(0.2);
    }


    public void stop() {
        motor.stopMotor();
    }

    public boolean hasItem() {
        return !sensor.get();
    }
    public void periodic(){
        SmartDashboard.putBoolean("HasItem", hasItem());
    }
}


