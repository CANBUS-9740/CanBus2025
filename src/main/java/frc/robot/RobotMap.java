package frc.robot;

import swervelib.parser.PIDFConfig;

public class RobotMap {

    private RobotMap() {}

    public static final double ROBOT_MASS_KG = 20;
    public static final double SWERVE_WIDTH = 0.71;
    public static final double SWERVE_LENGTH = 0.71;
    public static final double SWERVE_MAX_SPEED = 4;
    public static final double SWERVE_DRIVE_GEAR_RATIO = 6.75;
    public static final double SWERVE_DRIVE_WHEEL_RADIUS = 0.0508;
    public static final double SWERVE_STEER_GEAR_RATIO = 12.8;
    public static final double SWERVE_WHEEL_FRICTION_COEFFICIENT = 1.19; // stolen from yagsl
    public static final double SWERVE_OPTIMAL_VOLTAGE = 12;
    public static final int SWERVE_DRIVE_CURRENT_LIMIT = 40;
    public static final int SWERVE_STEER_CURRENT_LIMIT = 20;
    public static final double SWERVE_DRIVE_RAMP_RATE = 0.25;
    public static final double SWERVE_STEER_RAMP_RATE = 0.25;
    public static final double SWERVE_DRIVE_FRICTION_VOLTAGE = 0.2;
    public static final double SWERVE_STEER_FRICTION_VOLTAGE = 0.3;
    public static final double SWERVE_STEER_ROTATIONAL_INERTIA = 0.03;
    public static final PIDFConfig SWERVE_DRIVE_PIDF = new PIDFConfig(1, 0, 0, 0, 0);
    public static final PIDFConfig SWERVE_STEER_PIDF = new PIDFConfig(0.01, 0, 0, 0, 0);
    public static final PIDFConfig SWERVE_HEADING_PIDF = new PIDFConfig(0.4, 0, 0.01);
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
    public static final double SWERVE_FRONT_RIGHT_ZERO_ANGLE =  39.199;
    public static final double SWERVE_FRONT_LEFT_ZERO_ANGLE =  84.111;
    public static final double SWERVE_BACK_RIGHT_ZERO_ANGLE =  335.30;
    public static final double SWERVE_BACK_LEFT_ZERO_ANGLE =  196.34;

    //clawJoint
    public static final int CLAWJOINT_MOTOR = 0;
    public static final int LIMITSWITCH_CLAWJOINT_RIHGHT = 0;
    public static final int LIMITSWITCH_CLAWJOINT_LEFT = 0;
    public static final double PPR_CLAWJOINT = 1/1024;

}
