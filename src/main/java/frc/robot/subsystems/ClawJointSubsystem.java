package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;



public class ClawJointSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final AbsoluteEncoder absoluteEncoder;
    private final DigitalInput limitSwitchLeft;
    private final DigitalInput limitSwitchRight;
    private final SparkClosedLoopController controller;


    public ClawJointSubsystem(){
        motor = new SparkMax(RobotMap.CLAWJOINT_MOTOR, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pidf(0,0,0,0);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        controller = motor.getClosedLoopController();

        absoluteEncoder = motor.getAbsoluteEncoder();
        limitSwitchLeft = new DigitalInput(RobotMap.LIMITSWITCH_CLAWJOINT_LEFT);
        limitSwitchRight = new DigitalInput(RobotMap.LIMITSWITCH_CLAWJOINT_RIHGHT);


        config.limitSwitch
                .forwardLimitSwitchEnabled(true)
                .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

        config.softLimit
                .forwardSoftLimitEnabled(true)
                .forwardSoftLimit(100)
                .reverseSoftLimitEnabled(true)
                .reverseSoftLimit(0);

    }

    public boolean isPressedLeft(){
        return !limitSwitchLeft.get();
    }

    public boolean isPressedRight(){
        return !limitSwitchRight.get();
    }

    public double getPositionDegrees(){
       return absoluteEncoder.getPosition()/RobotMap.PPR_CLAWJOINT * 360;
    }

    public void moveToPosition(double positionDegrees){
        double targetPosition = getPositionDegrees() * RobotMap.PPR_CLAWJOINT / 360;
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
    }
}
