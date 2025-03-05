package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
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
        motor = new SparkMax(RobotMap.GRIPPER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        sensor = new DigitalInput(RobotMap.GRIPPER_SENSOR_ID);
    }

    public void collectItem() {
        motor.set(0.6);
    }

    public void releaseItem(){
        motor.set(-0.5);
    }

    public void releaseItemSlow() {
        motor.set(-0.15);
    }

    public void holdItem(){
        motor.set(0.1);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean hasItem() {
        return !sensor.get();
    }
    public void periodic(){
        SmartDashboard.putBoolean("ItemInClaw", hasItem());
        SmartDashboard.putNumber("outputAmper", motor.getOutputCurrent());
    }
}


