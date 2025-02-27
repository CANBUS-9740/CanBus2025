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
    private final Rev2mDistanceSensor distMXP;
    private final DigitalInput limit;


    public ClawGripperSystem() {
        motor = new SparkMax(RobotMap.CLAW_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        limit = new DigitalInput(RobotMap.ARM_TELESCOPIC_LIMIT_SWITCH);
        distMXP = new Rev2mDistanceSensor(Rev2mDistanceSensor.Port.kMXP);
        distMXP.setAutomaticMode(true);
        distMXP.setEnabled(true);
        distMXP.setMeasurementPeriod(0.01);
        distMXP.setDistanceUnits(Rev2mDistanceSensor.Unit.kMillimeters);
        distMXP.setRangeProfile(Rev2mDistanceSensor.RangeProfile.kDefault);
    }

    public void collectItem() {
        motor.set(0.8);
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
//        double distance = distMXP.getRange();
//        return distance >=RobotMap.CLAW_SENSOR_MIN_DISTANCE && distance <= RobotMap.CLAW_SENSOR_MAX_DISTANCE;
        return !limit.get();
    }
    public void periodic(){
        SmartDashboard.putNumber("DistanceSensorRange", distMXP.getRange());
        SmartDashboard.putBoolean("ItemInClaw", hasItem());
        SmartDashboard.putNumber("outputAmper", motor.getOutputCurrent());
        SmartDashboard.putBoolean("GetLimitSwitch", hasItem());
    }
}


