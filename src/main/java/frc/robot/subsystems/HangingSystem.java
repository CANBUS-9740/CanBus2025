package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import javax.swing.*;

public class HangingSystem extends SubsystemBase {

    private final Servo motorR;
    private final Servo motorL;

    public HangingSystem(){
        motorR =  new Servo(RobotMap.HANGING_MOTOR_RIGHT_ID);
        motorL =  new Servo(RobotMap.HANGING_MOTOR_LEFT_ID);
    }

    public void openRight(){
        motorR.setAngle(RobotMap.HANGING_OPEN_ANGLE);
    }

    public void openLeft(){
        motorL.setAngle(RobotMap.HANGING_OPEN_ANGLE);
    }

    public void closeLeft(){
        motorL.setAngle(RobotMap.HANGING_CLOSE_ANGLE);
    }

    public void closeRight(){
        motorR.setAngle(RobotMap.HANGING_CLOSE_ANGLE);
    }

    public boolean didGetToAngleLeft(double targetAngle){
        return MathUtil.isNear(targetAngle, motorL.getAngle(), 0.5);
    }

    public boolean didGetToAngleRight(double targetAngle){
        return MathUtil.isNear(targetAngle, motorR.getPosition(), 0.5);
    }

    public void stopLeft(){
        motorL.setDisabled();
    }

    public void stopRight(){
        motorR.setDisabled();
    }

    public void periodic(){
        SmartDashboard.putNumber("leftHangAngle:", motorL.getAngle());
        SmartDashboard.putNumber("rightHangAngle:", motorR.getAngle());
    }

}
