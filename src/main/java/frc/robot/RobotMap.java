package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import swervelib.parser.PIDFConfig;

public class RobotMap {

    private RobotMap() {
    }

    public static final double STAND_SELECTION_HEADING_MARGIN = 5;
    public static final double STAND_SELECTION_GENERAL_ORIENTATION_MARGIN = 45;

    public static final double SWERVE_WIDTH = 0.71;
    public static final double SWERVE_LENGTH = 0.71;
    public static final double SWERVE_DRIVE_WHEEL_RADIUS = 0.0508;
    public static final double SWERVE_DRIVE_GEAR_RATIO = 6.12;
    public static final double SWERVE_STEER_GEAR_RATIO = 150.0 / 7;
    public static final double SWERVE_MAX_SPEED = DCMotor.getKrakenX60(1).freeSpeedRadPerSec / Math.PI / SWERVE_DRIVE_GEAR_RATIO * (2 * Math.PI * SWERVE_DRIVE_WHEEL_RADIUS);
    public static final double SWERVE_DRIVE_RAMP_RATE = 0.25;
    public static final double SWERVE_STEER_RAMP_RATE = 0.25;
    public static final PIDFConfig SWERVE_DRIVE_PIDF = new PIDFConfig(2, 0, 0, 0, 0);
    public static final PIDFConfig SWERVE_STEER_PIDF = new PIDFConfig(0.01, 0, 0, 0, 0);
    public static final PIDFConfig SWERVE_HEADING_PIDF = new PIDFConfig(0.007448, 0, 0.00758, 0, 0);//new PIDFConfig(0.4, 0, 0.01,0);
    public static final int SWERVE_DRIVE_FRONT_LEFT_MOTOR_ID = 13;
    public static final int SWERVE_DRIVE_FRONT_RIGHT_MOTOR_ID = 10;
    public static final int SWERVE_DRIVE_BACK_LEFT_MOTOR_ID = 12;
    public static final int SWERVE_DRIVE_BACK_RIGHT_MOTOR_ID = 11;
    public static final int SWERVE_STEER_FRONT_LEFT_MOTOR_ID = 17;
    public static final int SWERVE_STEER_FRONT_RIGHT_MOTOR_ID = 16;
    public static final int SWERVE_STEER_BACK_LEFT_MOTOR_ID = 15;
    public static final int SWERVE_STEER_BACK_RIGHT_MOTOR_ID = 14;
    public static final int SWERVE_ENCODER_FRONT_LEFT_ID = 5;
    public static final int SWERVE_ENCODER_FRONT_RIGHT_ID = 6;
    public static final int SWERVE_ENCODER_BACK_LEFT_ID = 4;
    public static final int SWERVE_ENCODER_BACK_RIGHT_ID = 3;
    public static final int SWERVE_PIGEON_ID = 7;
    public static final double SWERVE_FRONT_RIGHT_ZERO_ANGLE = 79.628906;
    public static final double SWERVE_FRONT_LEFT_ZERO_ANGLE = 315.439453;
    public static final double SWERVE_BACK_RIGHT_ZERO_ANGLE = 278.789062;
    public static final double SWERVE_BACK_LEFT_ZERO_ANGLE = 336.357422;
    public static final double SWERVE_HEIGHT = 0;
    public static final double SWERVE_DISTANCE_FROM_SOURCE = 0;
    public static final double SWERVE_DISTANCE_FROM_CORAL = 0;
    public static final double SWERVE_DISTANCE_FROM_PROCESSOR = 0;

    // arm telescopic
    public static final int ARM_TELESCOPIC_MOTOR_ID = 37;
    public static final int ARM_TELESCOPIC_LIMIT_SWITCH = 0;
    public static final double ARM_TELESCOPIC_GEAR_RATIO = 25;
    public static final double ARM_TELESCOPIC_P = 1;
    public static final double ARM_TELESCOPIC_I = 0;
    public static final double ARM_TELESCOPIC_D = 0;
    public static final double ARM_TELESCOPIC_I_ZONE = 0;
    public static final double ARM_TELESCOPIC_FORWARD_SOFT_LIMIT = 0;
    public static final double ARM_TELESCOPIC_REVERSE_SOFT_LIMIT = 0;
    public static final double ARM_TELESCOPIC_DRUM_CIRCUMFERENSE = 0.11997676;
    public static final double ARM_TELESCOPIC_RESET_LENGTH_METERS = 0;
    public static final double ARM_TELESCOPIC_MAXIMUM_LENGTH = 180;
    public static final double ARM_TELESCOPIC_MINIMUM_LENGTH = 0;
    public static final double ARM_TELESCOPIC_LOWER_REEF_ALGAE_LENGTH = 0;
    public static final double ARM_TELESCOPIC_HIGH_REEF_ALGAE_LENGTH = 0;
    public static final double ARM_TELESCOPIC_BASE_LENGTH = 0.5;

    //ARM JOINT
    public static final int ARM_JOINT_MOTOR_ID_MASTER = 35;
    public static final int ARM_JOINT_MOTOR_ID_FOLLOWER = 40;
    public static final double ARM_JOINT_ZERO_ANGLE = 26.64;
    public static final double P_ARM_JOINT = 3.6;
    public static final double I_ARM_JOINT = 0;
    public static final double D_ARM_JOINT = 0;
    public static final double ARM_JOINT_ENCODER_ZERO_OFFSET = 0.5345518;
    public static final TrapezoidProfile.Constraints ARM_JOINT_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(400, 100);
    public static final double ARM_JOINT_POSITION_TOLERANCE = 1;
    public static final double ARM_JOINT_VELOCITY_TOLERANCE = 5;
    public static final double ARM_JOINT_KF = 0.0;
    public static final double ARM_JOINT_GEAR_RATIO = 144;
    public static final double ARM_JOINT_MINIMUM_ANGLE = 1;
    public static final double ARM_JOINT_MAXIMUM_ANGLE = 120;
    public static final double ARM_JOINT_FLOOR_ANGLE = 0;
    public static final double ARM_JOINT_UNDER_CAGE_ANGLE = 0;
    public static final double ARM_JOINT_LOWER_REEF_ALGAE_ANGLE = 0;
    public static final double ARM_JOINT_HIGH_REEF_ALGAE_ANGLE = 0;

    //claw
    public static final int CLAW_MOTOR_ID = 39;
    public static final double CLAW_GEAR_RATIO = 9;
    public static final int CLAW_SENSOR_MIN_DISTANCE = 70;
    public static final int CLAW_SENSOR_MAX_DISTANCE = 90;

    //clawJoint
    public static final int CLAWJOINT_MOTOR_ID = 38;
    public static final double CLAWJOINT_GEAR_RATIO = 125;
    public static final double P_CLAWJOINT = 2.5;
    public static final double I_CLAWJOINT = 0;
    public static final double D_CLAWJOINT = 0;
    public static final double CLAWJOINT_ABS_ENCODER_ZERO_OFFSET = 0.27;
    public static final double CLAWJOINT_VELOCITY_TOLERANCE = 5;
    public static final double CLAWJOINT_POSITION_TOLERANCE = 3;
    public static final double CLAWJOINT_MAXIMUM_ANGLE = 165.24;
    public static final double CLAWJOINT_MINIMUM_ANGLE = 3;
    public static final double CLAWJOINT_ZERO_ANGLE = 100;
    public static final double CLAWJOINT_KF = 0.01;
    public static final TrapezoidProfile.Constraints CLAWJOINT_MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(300, 200);


    //commandsGroup
    public static final double SOURCE_HEIGHT = 1.35;
    public static final double CORAL_PODIUM_POLE_HEIGHT = 0.46;
    public static final double CORAL_LOWER_POLE_HEIGHT = 0.81;
    public static final double CORAL_MEDIUM_POLE_HEIGHT = 1.21;
    public static final double CORAL_HIGH_POLE_HEIGHT = 1.83;
    public static final double PROCESSOR_PLACE_HEIGHT = 1.35;
    public static final double ROBOT_MAXIMUM_DISTANCE = 2;
    public static final double CALCULATION_COLLECT_FROM_SOURCE = Math.sqrt(Math.pow(RobotMap.SWERVE_DISTANCE_FROM_SOURCE,2) + Math.pow(RobotMap.SOURCE_HEIGHT-RobotMap.SWERVE_HEIGHT,2));
    public static final double ARM_LENGTH_PODIUM = Math.sqrt(Math.pow(RobotMap.SWERVE_DISTANCE_FROM_CORAL,2) + Math.pow(RobotMap.CORAL_PODIUM_POLE_HEIGHT-RobotMap.SWERVE_HEIGHT,2));
    public static final double ARM_LENGTH_FIRST = Math.sqrt(Math.pow(RobotMap.SWERVE_DISTANCE_FROM_CORAL,2) + Math.pow(RobotMap.CORAL_LOWER_POLE_HEIGHT-RobotMap.SWERVE_HEIGHT,2));
    public static final double ARM_LENGTH_SECOND = Math.sqrt(Math.pow(RobotMap.SWERVE_DISTANCE_FROM_CORAL,2) + Math.pow(RobotMap.CORAL_MEDIUM_POLE_HEIGHT-RobotMap.SWERVE_HEIGHT,2));
    public static final double ARM_LENGTH_THIRD = Math.sqrt(Math.pow(RobotMap.SWERVE_DISTANCE_FROM_CORAL,2) + Math.pow(RobotMap.CORAL_HIGH_POLE_HEIGHT-RobotMap.SWERVE_HEIGHT,2));
    public static final double ARM_LENGTH_FLOOR = 0;
    public static final double CALCULATION_PLACE_IN_PROCESSOR = Math.sqrt(Math.pow(RobotMap.SWERVE_DISTANCE_FROM_PROCESSOR,2) + Math.pow(RobotMap.PROCESSOR_PLACE_HEIGHT-RobotMap.SWERVE_HEIGHT,2));
    public static final double ANGLE_PROCESSOR = Math.atan(RobotMap.PROCESSOR_PLACE_HEIGHT-RobotMap.SWERVE_HEIGHT/RobotMap.SWERVE_DISTANCE_FROM_CORAL);
    public static final double ARM_JOINT_ANGLE_PODIUM = 77;
    public static final double ARM_JOINT_ANGLE_FIRST = 95.2;
    public static final double ARM_JOINT_ANGLE_SECOND = 0;
    public static final double ARM_JOINT_ANGLE_THIRD = 0;
    public static final double ARM_JOINT_ANGLE_SOURCE = 70;
    public static final double CLAWJOINT_SOURCE_ANGLE = 71;
    public static final double CLAWJOINT_FLOOR_ANGLE = 0;
    public static final double CLAWJOINT_CORAL_PODIUM_POLE_ANGLE = 6;
    public static final double CLAWJOINT_CORAL_FIRST_POLE_ANGLE = 12;
    public static final double CLAWJOINT_CORAL_SECOND_POLE_ANGLE = 0;
    public static final double CLAWJOINT_CORAL_THIRD_POLE_ANGLE = 0;
    public static final double CLAWJOINT_PROCESSOR_ANGLE = 0;
    public static final double CLAWJOINT_UNDER_CAGE_ANGLE = 0;
    public static final double CLAWJOINT_LOWER_REEF_ALGAE_ANGLE = 0;
    public static final double CLAWJOINT_HIGH_REEF_ALGAE_ANGLE = 0;
}
