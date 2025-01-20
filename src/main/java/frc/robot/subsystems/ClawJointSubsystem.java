package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;



public class ClawJointSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final AbsoluteEncoder absoluteEncoder;
    private final SparkClosedLoopController controller;
    private ClosedLoopConfig closedLoopConfig;


    public ClawJointSubsystem(){
        motor = new SparkMax(RobotMap.CLAWJOINT_MOTOR, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        controller = motor.getClosedLoopController();
        config.closedLoop
                .p(RobotMap.P_CLAWJOINT)
                .i(RobotMap.I_CLAWJOINT)
                .d(RobotMap.D_CLAWJOINT)
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder);

        absoluteEncoder = motor.getAbsoluteEncoder();
        config.absoluteEncoder
                .startPulseUs(RobotMap.CLAWJOINT_ENCODER_START_PULSE_U)
                .endPulseUs(RobotMap.CLAWJOINT_ENCODER_END_PULSE_US)
                .zeroOffset(RobotMap.ZERO_OFFSET);


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

    }

    public boolean isPressedForward(){
        return motor.getForwardLimitSwitch().isPressed();
    }

    public boolean isPressedReverse(){
        return motor.getReverseLimitSwitch().isPressed();
    }

    public double getPositionDegrees(){
       return absoluteEncoder.getPosition() * 360;
    }

    public void moveToPosition(double positionDegrees){
        double targetPosition = getPositionDegrees() / 360;
        controller.setReference(targetPosition, SparkBase.ControlType.kPosition);
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("PositionDegrees",getPositionDegrees());
        SmartDashboard.putBoolean("ForwardLimitSwitch", isPressedForward());
        SmartDashboard.putBoolean("ReverseLimitSwitch", isPressedReverse());
    }
}
