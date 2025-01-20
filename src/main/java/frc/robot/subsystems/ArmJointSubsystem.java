package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ArmJointSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final AbsoluteEncoder encoder;
    private final SparkClosedLoopController pidController;

    public ArmJointSubsystem() {
        motor = new SparkMax(RobotMap.ARM_JOINT, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);

        config.absoluteEncoder
                .startPulseUs(RobotMap.START_PULSE_US)
                .endPulseUs(RobotMap.END_PULSE_US)
                .zeroOffset(RobotMap.ZERO_OFFSET);

        config.closedLoop
                .p(RobotMap.P_ARM_JOINT)
                .i(RobotMap.I_ARM_JOINT)
                .d(RobotMap.D_ARM_JOINT)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        config.limitSwitch
                .forwardLimitSwitchEnabled(true)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

        config.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(0)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(0);

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        encoder = motor.getAbsoluteEncoder();
        pidController = motor.getClosedLoopController();
    }

    public double getPositionDegrees(){
        return encoder.getPosition()*360;
    }

    public void moveToPosition(double positionDegrees){
        if(positionDegrees<=RobotMap.ARM_JOINT_MAX_ANGLE && positionDegrees>=RobotMap.ARM_JOINT_MIN_ANGLE)
            pidController.setReference(positionDegrees, SparkBase.ControlType.kPosition);
    }

    public void move(double speed){
        if(getPositionDegrees()>= RobotMap.ARM_JOINT_MAX_ANGLE || getPositionDegrees()<= RobotMap.ARM_JOINT_MIN_ANGLE)
            motor.stopMotor();
        else
            motor.set(speed);
    }

    public void raise(){
        motor.set(0.8);
    }

    public void lower(){
        motor.set(-0.5);
    }

    public void hold(){
        motor.set(0.2);
    }

    public void stop(){
        motor.stopMotor();
    }

    private void updateShuffleBoard() {
        SmartDashboard.putNumber("getPositionDegrees", getPositionDegrees());
        SmartDashboard.putBoolean("forwardHardLimits", motor.getForwardLimitSwitch().isPressed());
        SmartDashboard.putBoolean("reverseHardLimits", motor.getReverseLimitSwitch().isPressed());
    }

    public void periodic(){
        updateShuffleBoard();
    }


}
