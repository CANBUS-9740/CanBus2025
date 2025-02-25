package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ClawJointSystem extends SubsystemBase {

    private final SparkMax motor;
    private final AbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder relativeEncoder;
    private final SparkClosedLoopController controller;

    public ClawJointSystem(){
        motor = new SparkMax(RobotMap.CLAWJOINT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1 / RobotMap.CLAWJOINT_GEAR_RATIO)
                .velocityConversionFactor(1 / RobotMap.CLAWJOINT_GEAR_RATIO);
        config.closedLoop
                .p(RobotMap.P_CLAWJOINT)
                .i(RobotMap.I_CLAWJOINT)
                .d(RobotMap.D_CLAWJOINT)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        config.absoluteEncoder
                .inverted(true)
                .zeroOffset(RobotMap.CLAWJOINT_ABS_ENCODER_ZERO_OFFSET);
        config.limitSwitch
                .forwardLimitSwitchEnabled(false)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(false)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
        config.softLimit
                .forwardSoftLimitEnabled(false)
                .forwardSoftLimit(0)
                .reverseSoftLimitEnabled(false)
                .reverseSoftLimit(0);

        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        controller = motor.getClosedLoopController();
        absoluteEncoder = motor.getAbsoluteEncoder();
        relativeEncoder = motor.getEncoder();
    }

    public boolean isPressedForward(){
        return motor.getForwardLimitSwitch().isPressed();
    }

    public boolean isPressedReverse(){
        return motor.getReverseLimitSwitch().isPressed();
    }

    public double getRawPositionDegrees(){
       return absoluteEncoder.getPosition() * 360;
    }

    public double getLogicalPositionDegrees(){
        return getRawPositionDegrees() - RobotMap.CLAWJOINT_ZERO_ANGLE;
    }

    public double getVelocityRPM() {
        return relativeEncoder.getVelocity();
    }

    public void moveToPosition(double positionDegrees, double armAngle) {
        double targetPosition = positionDegrees / 360.0;
        double ff = Math.cos(Math.toRadians(getLogicalPositionDegrees() + armAngle)) * RobotMap.CLAWJOINT_KF;
        controller.setReference(targetPosition, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }

    public void move(double speed){
        motor.set(speed);
    }

    public void raise(){
        move(0.8);
    }

    public void lower(){
        move(-0.5);
    }

    public void hold(){
        move(0.2);
    }

    public void stop(){
        motor.stopMotor();
    }

    public boolean didReachPosition(double targetAngle) {
        return MathUtil.isNear(targetAngle, getRawPositionDegrees(), RobotMap.CLAWJOINT_POSITION_TOLERANCE) &&
                Math.abs(getVelocityRPM()) < RobotMap.CLAWJOINT_VELOCITY_TOLERANCE;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClawJointRawPosition", getRawPositionDegrees());
        SmartDashboard.putNumber("ClawJointLogicalPosition", getLogicalPositionDegrees());
        SmartDashboard.putBoolean("ForwardLimitSwitch", isPressedForward());
        SmartDashboard.putBoolean("ReverseLimitSwitch", isPressedReverse());
    }
}
