package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class HangSystem extends SubsystemBase {
    private SparkMax motor;
    private DigitalInput limitSwitchBottom;
    private DigitalInput limitSwitchTop;


    public HangSystem(){
        motor = new SparkMax(RobotMap.HANGING_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        limitSwitchBottom = new DigitalInput(RobotMap.HANGING_LIMIT_SWITCH_BOTTOM);
        limitSwitchTop = new DigitalInput(RobotMap.HANGING_LIMIT_SWITCH_TOP);
    }
}
