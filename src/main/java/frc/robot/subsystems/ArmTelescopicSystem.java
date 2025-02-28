package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ArmTelescopicSystem extends SubsystemBase {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final DigitalInput limitSwitch;
    private final SparkClosedLoopController pid;

    public ArmTelescopicSystem() {
        motor = new SparkMax(RobotMap.ARM_TELESCOPIC_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        limitSwitch = new DigitalInput(RobotMap.ARM_TELESCOPIC_LIMIT_SWITCH);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kBrake);
        config.inverted(false);
        config.softLimit
                        .forwardSoftLimitEnabled(false)
                        .forwardSoftLimit(RobotMap.ARM_TELESCOPIC_FORWARD_SOFT_LIMIT)
                        .reverseSoftLimitEnabled(false)
                        .reverseSoftLimit(RobotMap.ARM_TELESCOPIC_REVERSE_SOFT_LIMIT);
        config.closedLoop
                        .p(RobotMap.ARM_TELESCOPIC_P)
                        .i(RobotMap.ARM_TELESCOPIC_I)
                        .d(RobotMap.ARM_TELESCOPIC_D)
                        .iZone(RobotMap.ARM_TELESCOPIC_I_ZONE);
        config.encoder.positionConversionFactor(1 / RobotMap.ARM_TELESCOPIC_GEAR_RATIO).velocityConversionFactor(1 / RobotMap.ARM_TELESCOPIC_GEAR_RATIO);

        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        encoder = motor.getEncoder();
        pid = motor.getClosedLoopController();
    }

    public double getLengthMeters() {
        return (encoder.getPosition() * RobotMap.ARM_TELESCOPIC_DRUM_CIRCUMFERENSE) + RobotMap.ARM_TELESCOPIC_BASE_LENGTH;
    }

    public double getMeasuredLengthMeters() {
        return (encoder.getPosition() * RobotMap.ARM_TELESCOPIC_DRUM_CIRCUMFERENSE);
    }

    public void moveToLength(double lengthMeters) {
        double position = (lengthMeters - RobotMap.ARM_TELESCOPIC_BASE_LENGTH) / RobotMap.ARM_TELESCOPIC_DRUM_CIRCUMFERENSE;
        pid.setReference(position, SparkBase.ControlType.kPosition);
    }

    public void setEncoderValue(double value) {
        encoder.setPosition(value);
    }

    public void move(double speed){
        motor.set(speed);
    }

    public void extend() {
        motor.set(0.3);
    }

    public void retract() {
        motor.set(-0.3);
    }

    public void hold() {
        motor.set(0.2);
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean didReach(double targetLength) {
        return MathUtil.isNear(targetLength, getLengthMeters(), 0.1) && Math.abs(encoder.getVelocity()) < 5;
    }

    public double calculateLengthForTarget(double distance, double height) {
        return Math.sqrt(Math.pow(distance, 2) + Math.pow(height, 2));
    }

    public boolean getResetLimitSwitch() {
        return !limitSwitch.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ArmTelescopicLengthEncoder", getMeasuredLengthMeters());
        SmartDashboard.putNumber("ArmTelescopicLength", getLengthMeters());
        SmartDashboard.putBoolean("ArmTelescopicLimitSwitch", getResetLimitSwitch());
    }
}
