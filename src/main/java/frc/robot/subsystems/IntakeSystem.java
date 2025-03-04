package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSystem extends SubsystemBase {

    private final SparkMax motor;
    private final DigitalInput limitSwitch;


    public IntakeSystem() {
        motor = new SparkMax(RobotMap.INTAKE_MOTOR, SparkLowLevel.MotorType.kBrushless);
        limitSwitch = new DigitalInput(RobotMap.INTAKE_SWITCH);


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
        return !limitSwitch.get();
        }
    public void periodic(){
        SmartDashboard.putBoolean("InIntake", hasItem());
        SmartDashboard.putNumber("outputIntake", motor.getOutputCurrent());
    }
}


