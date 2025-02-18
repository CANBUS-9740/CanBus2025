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
                .startPulseUs(RobotMap.ARM_JOINT_ENCODER_START_PULSE_US)
                .endPulseUs(RobotMap.ARM_JOINT_ENCODER_END_PULSE_US)
                .zeroOffset(RobotMap.ARM_JOINT_ENCODER_ZERO_OFFSET);

        config.closedLoop
                .p(RobotMap.P_ARM_JOINT)
                .i(RobotMap.I_ARM_JOINT)
                .d(RobotMap.D_ARM_JOINT)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);

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

        masterMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        absoluteEncoder = masterMotor.getAbsoluteEncoder();
        relativeEncoder = masterMotor.getEncoder();
        pidController = masterMotor.getClosedLoopController();

        relativeEncoder.setPosition(absoluteEncoder.getPosition());

        config = new SparkMaxConfig();
        config.follow(masterMotor, true);
        followerMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    public double getPositionDegrees(){
        return relativeEncoder.getPosition() * 360;
    }

    public void moveToPosition(double positionDegrees){
        double ff = Math.cos(Math.toRadians(getPositionDegrees())) * RobotMap.ARM_JOINT_KF;
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
        return MathUtil.isNear(targetPosition, getPositionDegrees(), RobotMap.ARM_JOINT_POSITION_TOLERANCE) &&
                Math.abs(relativeEncoder.getVelocity()) <= RobotMap.ARM_JOINT_VELOCITY_TOLERANCE;
    }

    public double calculateAngleForTarget(double distance, double height) {
        return Math.toDegrees(Math.atan(height / distance));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ArmJointPosition", getPositionDegrees());
        SmartDashboard.putBoolean("ArmJointForwardLimit", masterMotor.getForwardLimitSwitch().isPressed());
        SmartDashboard.putBoolean("ArmJointReverseLimit", masterMotor.getReverseLimitSwitch().isPressed());
    }
}
