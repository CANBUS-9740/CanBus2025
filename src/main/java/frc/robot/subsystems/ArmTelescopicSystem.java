package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ArmTelescopicSystem extends SubsystemBase {
    private SparkMax motor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController pid;

    public ArmTelescopicSystem() {
        motor = new SparkMax(RobotMap.ARM_TELESCOPIC_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        encoder = motor.getAlternateEncoder();
        pid = motor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(SparkBaseConfig.IdleMode.kCoast);
        config.inverted(false);
        config.softLimit
                        .forwardSoftLimitEnabled(true)
                        .forwardSoftLimit(RobotMap.ARM_TELESCOPIC_FORWARD_SOFT_LIMIT)
                        .reverseSoftLimitEnabled(true)
                        .reverseSoftLimit(RobotMap.ARM_TELESCOPIC_REVERSE_SOFT_LIMIT);
        config.closedLoop
                        .p(RobotMap.ARM_TELESCOPIC_P)
                        .i(RobotMap.ARM_TELESCOPIC_I)
                        .d(RobotMap.ARM_TELESCOPIC_D)
                        .iZone(RobotMap.ARM_TELESCOPIC_I_ZONE);
        motor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
    }

    public double getLengthMeter() {
        return encoder.getPosition() / RobotMap.ARM_TELESCOPIC_GEAR_RATIO * RobotMap.ARM_TELESCOPIC_DRUM_CIRCUMFERENSE;
    }

    public void moveToLength(double lengthMeters){
        pid.setReference(RobotMap.ARM_TELESCOPIC_GEAR_RATIO * RobotMap.ARM_TELESCOPIC_DRUM_CIRCUMFERENSE / lengthMeters, SparkBase.ControlType.kPosition);
    }

    public void move(double speed){
        motor.set(speed);
    }

    public void extend() {
        motor.set(-0.2);
    }

    public void retract() {
        motor.set(0.5);
    }

    public void hold() {
        motor.set(0.2);
    }

    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("getLengthArmInMeters", getLengthMeter());
    }
}
