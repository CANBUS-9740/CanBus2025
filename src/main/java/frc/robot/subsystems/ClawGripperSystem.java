package frc.robot.subsystems;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClawGripperSystem extends SubsystemBase {

    private final SparkMax motor;
    private final SparkLimitSwitch limitSwitch;

    public ClawGripperSystem() {
        motor = new SparkMax(RobotMap.GRIPPER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        config.inverted(true);
        config.limitSwitch
                .forwardLimitSwitchEnabled(true)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(false)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyClosed);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        limitSwitch = motor.getForwardLimitSwitch();
    }

    public void collectItem() {
        motor.set(0.4);
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
        return limitSwitch.isPressed();
    }

    public void periodic(){
        SmartDashboard.putBoolean("ItemInClaw", hasItem());
        SmartDashboard.putNumber("outputAmper", motor.getOutputCurrent());
    }
}


