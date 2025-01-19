package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.RobotMap;

public class ArmJointSubsystem {
    private SparkMax motor;
    private AbsoluteEncoder encoder;
    private SparkClosedLoopController pidController;
    private ClosedLoopConfig closedLoop;

    public ArmJointSubsystem() {
        motor = new SparkMax(RobotMap.ARM_JOINT, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();

        encoder = motor.getAbsoluteEncoder();
        config.encoder.countsPerRevolution(RobotMap.CPR);

        pidController = motor.getClosedLoopController();
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
                .forwardSoftLimit()
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit();

    }

    public double getPositionDegrees(){
        return encoder.getPosition()*360;
    }

    public void moveToPosition(double positionDegrees, double speed, double power){
        if(positionDegrees<=RobotMap.ARM_JOINT_MAX_ANGLE && positionDegrees>=RobotMap.ARM_JOINT_MIN_ANGLE)
            pidController.setReference(positionDegrees, SparkBase.ControlType.kPosition);
    }

    public void move(double speed, double power){
        if(power>0 && getPositionDegrees()>= RobotMap.ARM_JOINT_MAX_ANGLE || power>0 && getPositionDegrees()<= RobotMap.ARM_JOINT_MIN_ANGLE)
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

    public void hold(int targetAngle){

    }

    public void stop(){
        motor.stopMotor();
    }

}
