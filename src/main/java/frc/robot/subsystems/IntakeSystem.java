package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSystem extends SubsystemBase {

    private final SparkMax motor;
    private final DigitalInput limitswitch;


    public IntakeSystem() {
        motor = new SparkMax(RobotMap.CLAW_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        limitswitch = new DigitalInput(0);


    }

    public void collectItem() {
        motor.set(0.8);
    }

    public void releaseItem(){
        motor.set(-0.25);
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
        return !limitswitch.get();
        }
    public void periodic(){
        SmartDashboard.putBoolean("ItemInClaw", hasItem());
        SmartDashboard.putNumber("outputAmper", motor.getOutputCurrent());
    }
}


