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
        sparkMaxConfig.softLimit
                .forwardSoftLimitEnabled(false)
                .forwardSoftLimit(RobotMap.SOFT_LIMITS_FORWARD_HANG)
                .reverseSoftLimitEnabled(false)
                .reverseSoftLimit(RobotMap.SOFT_LIMITS_REVERSE_HANG);
        sparkMaxConfig.limitSwitch
                .forwardLimitSwitchEnabled(false)
                .reverseLimitSwitchEnabled(false);
        sparkMaxConfig.absoluteEncoder
                .zeroOffset(RobotMap.HANGING_ROBOT_ENCODER_OFFSET)
                .inverted(true)
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        sparkMaxConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);
        sparkMaxConfig.smartCurrentLimit(105);

        motor.configure(sparkMaxConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }

    public void toCage(){
        motor.set(-0.3);
    }

    public void toRobotFast() {
        motor.set(0.6);
    }

    public void toRobot(){
        motor.set(0.2);
    }

    public void stop(){
        motor.stopMotor();
    }

    public double getAbsoluteEncoder(){
        return absoluteEncoder.getPosition() * 360;
    }

    public boolean reachedPosition(double targetPosition) {
        return MathUtil.isNear(targetPosition, getAbsoluteEncoder(), RobotMap.HANGING_POSITION_TOLERANCE);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("reachedCage:", reachedPosition(RobotMap.HANGING_CAGE_ANGLE));
        SmartDashboard.putBoolean("reachedFold:", reachedPosition(RobotMap.HANGING_ROBOT_ANGLE));
        SmartDashboard.putNumber("absEncoderPosition:", getAbsoluteEncoder());
    }
}
