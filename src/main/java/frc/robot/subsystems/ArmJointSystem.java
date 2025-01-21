package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ArmJointSystem extends SubsystemBase {

    private final SparkMax motor;
    private final AbsoluteEncoder encoder;
    private final SparkClosedLoopController pidController;

    public ArmJointSystem() {
        motor = new SparkMax(RobotMap.ARMJOINT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);

        config.absoluteEncoder
                .startPulseUs(RobotMap.ARMJOINT_ENCODER_START_PULSE_US)
                .endPulseUs(RobotMap.ARMJOINT_ENCODER_END_PULSE_US)
                .zeroOffset(RobotMap.ARMJOINT_ENCODER_ZERO_OFFSET);

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
        return encoder.getPosition() * 360;
    }

    public void moveToPosition(double positionDegrees, double ff){
        pidController.setReference(positionDegrees, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }

    public void resetPID() {
        pidController.setReference(0, SparkBase.ControlType.kPosition);
    }

    public double getVelocity() {
        return encoder.getVelocity();
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

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ArmJointPosition", getPositionDegrees());
        SmartDashboard.putBoolean("ArmJointForwardLimit", motor.getForwardLimitSwitch().isPressed());
        SmartDashboard.putBoolean("ArmJointReverseLimit", motor.getReverseLimitSwitch().isPressed());
    }
}
