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

public class ArmJointSystem extends SubsystemBase {

    private final SparkMax masterMotor;
    private final SparkMax followerMotor;
    private final AbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder relativeEncoder;
    private final SparkClosedLoopController pidController;


    public ArmJointSystem() {
        masterMotor = new SparkMax(RobotMap.ARM_JOINT_MOTOR_ID_MASTER, SparkLowLevel.MotorType.kBrushless);
        followerMotor = new SparkMax(RobotMap.ARM_JOINT_MOTOR_ID_FOLLOWER, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1 / RobotMap.ARM_JOINT_GEAR_RATIO)
                .velocityConversionFactor(1 / RobotMap.ARM_JOINT_GEAR_RATIO);
        config.absoluteEncoder
                .zeroOffset(RobotMap.ARM_JOINT_ENCODER_ZERO_OFFSET);
        config.closedLoop
                .p(RobotMap.P_ARM_JOINT)
                .i(RobotMap.I_ARM_JOINT)
                .d(RobotMap.D_ARM_JOINT)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
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

        masterMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        absoluteEncoder = masterMotor.getAbsoluteEncoder();
        relativeEncoder = masterMotor.getEncoder();
        pidController = masterMotor.getClosedLoopController();


        config = new SparkMaxConfig();
        config.follow(masterMotor, true);
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        followerMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    public double getRawPositionDegrees(){
        return absoluteEncoder.getPosition() * 360;
    }

    public double getLogicalPositionDegrees() {
        return getRawPositionDegrees() - RobotMap.ARM_JOINT_ZERO_ANGLE;
    }

    public double getVelocityRpm() {
        return relativeEncoder.getVelocity();
    }

    public void move(double speed){
        masterMotor.set(speed);
    }

    public void moveToPosition(double positionDegrees, double armLength) {
        double kf = MathUtil.interpolate(0.035, 0.045, armLength / 0.65);
        double ff = Math.cos(Math.toRadians(getLogicalPositionDegrees())) * kf;
        SmartDashboard.putNumber("ArmJointKf", kf);
        SmartDashboard.putNumber("ArmJointff", ff);

        pidController.setReference(positionDegrees / 360.0, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }

    public void raise(){
        masterMotor.set(0.8);
    }

    public void lower(){
        masterMotor.set(-0.5);
    }

    public void hold(){
        masterMotor.set(0.2);
    }

    public void stop(){
        masterMotor.stopMotor();
    }

    public boolean reachedPosition(double targetPosition) {
        return MathUtil.isNear(targetPosition, getRawPositionDegrees(), RobotMap.ARM_JOINT_POSITION_TOLERANCE) &&
                Math.abs(getVelocityRpm()) <= RobotMap.ARM_JOINT_VELOCITY_TOLERANCE;
    }

    public double calculateAngleForTarget(double distance, double height) {
        return Math.toDegrees(Math.atan(height / distance));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ArmJointRawPosition", getRawPositionDegrees());
        SmartDashboard.putNumber("ArmJointLogicalPosition", getLogicalPositionDegrees());
        SmartDashboard.putBoolean("ArmJointForwardLimit", masterMotor.getForwardLimitSwitch().isPressed());
        SmartDashboard.putBoolean("ArmJointReverseLimit", masterMotor.getReverseLimitSwitch().isPressed());
        SmartDashboard.putNumber("ArmJointVelocity", getVelocityRpm());
    }
}
