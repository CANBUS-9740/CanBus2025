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

public class HangSystem extends SubsystemBase {
    private SparkMax motor;
    private AbsoluteEncoder absoluteEncoder;
    private RelativeEncoder relativeEncoder;
    SparkClosedLoopController pidController;


    public HangSystem(){
        motor = new SparkMax(RobotMap.HANGING_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        relativeEncoder = motor.getEncoder();
        absoluteEncoder = motor.getAbsoluteEncoder();
        pidController = motor.getClosedLoopController();

        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        sparkMaxConfig.closedLoop
                .p(RobotMap.HANGING_P)
                .i(RobotMap.HANGING_I)
                .d(RobotMap.HANGING_D)
                .iZone(0)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);
        sparkMaxConfig.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(RobotMap.SOFT_LIMITS_FORWARD_HANG);
        sparkMaxConfig.softLimit
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(RobotMap.SOFT_LIMITS_REVERSE_HANG);
        sparkMaxConfig.absoluteEncoder
                .zeroOffset(0)
                .inverted(false);
        sparkMaxConfig.encoder
                .positionConversionFactor(1 / RobotMap.HANGING_GEAR_RATIO)
                .velocityConversionFactor(1 / RobotMap.HANGING_GEAR_RATIO);
        sparkMaxConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);

        motor.configure(sparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

    }

    public void toCage(){
        motor.set(0.3);
    }

    public void toRobot(){
        motor.set(-0.8);
    }

    public void stop(){
        motor.stopMotor();
    }

    public double getAbsoluteEncoder(){
        return absoluteEncoder.getPosition() * 360;
    }

    public void moveToPosition(double positionDegrees) {
        double ff = Math.cos(Math.toRadians(getAbsoluteEncoder())) * RobotMap.HANGING_KF;
        SmartDashboard.putNumber("Hanging_ff", ff);

        pidController.setReference(positionDegrees / 360, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, ff, SparkClosedLoopController.ArbFFUnits.kPercentOut);
    }

    public boolean reachedPosition(double targetPosition) {
        return MathUtil.isNear(targetPosition, getAbsoluteEncoder(), RobotMap.HANGING_POSITION_TOLERANCE) &&
                Math.abs(getAbsoluteEncoder()) <= RobotMap.HANGING_VELOCITY_TOLERANCE;
    }



    @Override
    public void periodic() {
        SmartDashboard.putBoolean("reachedCage:", reachedPosition(RobotMap.HANGING_CAGE_ANGLE));
        SmartDashboard.putBoolean("reachedFold:", reachedPosition(RobotMap.HANGING_ROBOT_ANGLE));
        SmartDashboard.putNumber("absEncoderPosition:", getAbsoluteEncoder());
    }
}
