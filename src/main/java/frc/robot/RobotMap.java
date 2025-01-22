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
    public static final double SWERVE_DRIVE_RAMP_RATE = 0.25;
    public static final double SWERVE_STEER_RAMP_RATE = 0.25;
    public static final PIDFConfig SWERVE_DRIVE_PIDF = new PIDFConfig(1, 0, 0.001, 20, 0); // new PIDFConfig(1, 0, 0, 0, 0);//
    public static final PIDFConfig SWERVE_STEER_PIDF = new PIDFConfig(0.07, 0, 2.5 , 0, 0); // new PIDFConfig(0.01, 0, 0, 0, 0);//
    public static final PIDFConfig SWERVE_HEADING_PIDF =  new PIDFConfig(0.00749, 0, 0.00758,0,0);//new PIDFConfig(0.85, 0, 0.0250,0);
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
    public static final double SWERVE_FRONT_RIGHT_ZERO_ANGLE =79.541016;//  0.527344;
    public static final double SWERVE_FRONT_LEFT_ZERO_ANGLE = 68.291016;// 88.769531;
    public static final double SWERVE_BACK_RIGHT_ZERO_ANGLE =  277.207031;//175.078125;
    public static final double SWERVE_BACK_LEFT_ZERO_ANGLE =  226.582031;// 199.072266;
}
