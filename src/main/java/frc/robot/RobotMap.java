package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import swervelib.parser.PIDFConfig;

public class RobotMap {

    private RobotMap() {}

    public static final double ROBOT_MASS_KG = 20;
    public static final double SWERVE_WIDTH = 0.71;
    public static final double SWERVE_LENGTH = 0.71;
    public static final double SWERVE_DRIVE_WHEEL_RADIUS = 0.0508;
    public static final double SWERVE_DRIVE_GEAR_RATIO = 6.75;
    public static final double SWERVE_STEER_GEAR_RATIO = 150.0 / 7;
    public static final double SWERVE_MAX_SPEED = DCMotor.getKrakenX60(1).freeSpeedRadPerSec / Math.PI / SWERVE_DRIVE_GEAR_RATIO * (2 * Math.PI * SWERVE_DRIVE_WHEEL_RADIUS);
    public static final double SWERVE_WHEEL_FRICTION_COEFFICIENT = 1.19; // stolen from yagsl
    public static final double SWERVE_OPTIMAL_VOLTAGE = 12;
    public static final int SWERVE_DRIVE_CURRENT_LIMIT = 40;
    public static final int SWERVE_STEER_CURRENT_LIMIT = 20;
    public static final double SWERVE_DRIVE_RAMP_RATE = 0.25;
    public static final double SWERVE_STEER_RAMP_RATE = 0.25;
    public static final double SWERVE_DRIVE_FRICTION_VOLTAGE = 0.2;
    public static final double SWERVE_STEER_FRICTION_VOLTAGE = 0.3;
    public static final double SWERVE_STEER_ROTATIONAL_INERTIA = 0.03;
    public static final PIDFConfig SWERVE_DRIVE_PIDF = new PIDFConfig(2, 0, 0, 0, 0); // new PIDFConfig(1, 0, 0, 0, 0);//
    public static final PIDFConfig SWERVE_STEER_PIDF = new PIDFConfig(0.01, 0, 0, 0, 3);//new PIDFConfig(0.07, 0, 2.5, 0, 0); // new PIDFConfig(0.01, 0, 0, 0, 0);//
    public static final PIDFConfig SWERVE_HEADING_PIDF =  new PIDFConfig(0.4, 0, 0.01);//new PIDFConfig(0.85, 0, 0.0250,0);
    public static final int SWERVE_DRIVE_FRONT_LEFT_MOTOR_ID = 12;
    public static final int SWERVE_DRIVE_FRONT_RIGHT_MOTOR_ID = 10;
    public static final int SWERVE_DRIVE_BACK_LEFT_MOTOR_ID = 13;
    public static final int SWERVE_DRIVE_BACK_RIGHT_MOTOR_ID = 11;
    public static final int SWERVE_STEER_FRONT_LEFT_MOTOR_ID = 17;
    public static final int SWERVE_STEER_FRONT_RIGHT_MOTOR_ID = 16;
    public static final int SWERVE_STEER_BACK_LEFT_MOTOR_ID = 15;
    public static final int SWERVE_STEER_BACK_RIGHT_MOTOR_ID = 14;
    public static final int SWERVE_ENCODER_FRONT_LEFT_ID = 4;
    public static final int SWERVE_ENCODER_FRONT_RIGHT_ID = 6;
    public static final int SWERVE_ENCODER_BACK_LEFT_ID = 5;
    public static final int SWERVE_ENCODER_BACK_RIGHT_ID = 3;
    public static final int SWERVE_PIGEON_ID = 7;
    //zero angles
    public static final double SWERVE_FRONT_RIGHT_ZERO_ANGLE =222.714844;//  0.527344;
    public static final double SWERVE_FRONT_LEFT_ZERO_ANGLE = 168.134766;// 88.769531;
    public static final double SWERVE_BACK_RIGHT_ZERO_ANGLE =  215.859375;//175.078125;
    public static final double SWERVE_BACK_LEFT_ZERO_ANGLE =  313.242188;// 199.072266;
}
