package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import swervelib.parser.PIDFConfig;

import java.util.Optional;

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
    public static final double SWERVE_FRONT_RIGHT_ZERO_ANGLE = 79.541016;
    public static final double SWERVE_FRONT_LEFT_ZERO_ANGLE = 68.291016;
    public static final double SWERVE_BACK_RIGHT_ZERO_ANGLE = 277.207031;
    public static final double SWERVE_BACK_LEFT_ZERO_ANGLE = 226.582031;

    // arm telescopic
    public static final int ARM_TELESCOPIC_MOTOR_ID = 0;
    public static final double ARM_TELESCOPIC_GEAR_RATIO = 1;
    public static final double ARM_TELESCOPIC_P = 0;
    public static final double ARM_TELESCOPIC_I = 0;
    public static final double ARM_TELESCOPIC_D = 0;
    public static final double ARM_TELESCOPIC_I_ZONE = 0;
    public static final double ARM_TELESCOPIC_FORWARD_SOFT_LIMIT = 0;
    public static final double ARM_TELESCOPIC_REVERSE_SOFT_LIMIT = 0;
    public static final double ARM_TELESCOPIC_DRUM_CIRCUMFERENSE = 1;
    public static final double ARM_TELESCOPIC_RESET_LENGTH_METERS = 0;
    public static final double ARM_TELESCOPIC_MAXIMUM_LENGTH = 180;
    public static final double ARM_TELESCOPIC_MINIMUM_LENGTH = 0;

    //ARM JOINT
    public static final int ARM_JOINT_MOTOR_ID = 1;
    public static final double P_ARM_JOINT = 0;
    public static final double I_ARM_JOINT = 0;
    public static final double D_ARM_JOINT = 0;
    public static final int ARM_JOINT_ENCODER_START_PULSE_US = 1;
    public static final int ARM_JOINT_ENCODER_END_PULSE_US = 1024;
    public static final double ARM_JOINT_ENCODER_ZERO_OFFSET = 0;
    public static final TrapezoidProfile.Constraints MOTION_PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(RobotMap.MAX_VELOCITY, RobotMap.MAX_ACCELERATION);
    public static final double MAX_VELOCITY = 1000;
    public static final double MAX_ACCELERATION = 200;
    public static final double ARM_JOINT_POSITION_TOLERANCE = 2;
    public static final double ARM_JOINT_VELOCITY_TOLERANCE = 5;
    public static final double ARM_JOINT_KF = 0.1;
    public static final double ARM_JOINT_DEFAULT_POSITION = 90;
    public static final double ARM_JOINT_GEAR_RATIO = 1;
    public static final double ARM_JOINT_MINIMUM_ANGLE = 0;
    public static final double ARM_JOINT_MAXIMUM_ANGLE = 180;
    public static final double ARM_JOINT_FLOOR_ANGLE = 0;

    //claw
    public static final int CLAW_MOTOR_ID = 2;
    public static final int CLAW_SWITCH_PORT = 1;
    public static final int CLAW_SENSOR_MIN_DISTANCE = 70;
    public static final int CLAW_SENSOR_MAX_DISTANCE = 90;

    //clawJoint
    public static final int CLAWJOINT_MOTOR_ID = 3;
    public static final double P_CLAWJOINT = 0;
    public static final double I_CLAWJOINT = 0;
    public static final double D_CLAWJOINT = 0;
    public static final int CLAWJOINT_ABS_ENCODER_START_PULSE_US = 1;
    public static final int CLAWJOINT_ABS_ENCODER_END_PULSE_US = 1024;
    public static final int CLAWJOINT_ABS_ENCODER_ZERO_OFFSET = 0;
    public static final double CLAWJOINT_SOURCE_ANGLE = 0;
    public static final double CLAWJOINT_FLOOR_ANGLE = 0;
    public static final double CLAWJOINT_CORAL_PODIUM_POLE_ANGLE = 0;
    public static final double CLAWJOINT_CORAL_LOWER_POLE_ANGLE = 0;
    public static final double CLAWJOINT_CORAL_MEDIUM_POLE_ANGLE = 0;
    public static final double CLAWJOINT_CORAL_HIGH_POLE_ANGLE = 0;
    public static final double CLAWJOINT_PROCESSOR_ANGLE = 0;

    //commandsGroup
    public static final Pose2d POSE_SOURCE_A_BLUE = new Pose2d(0.851, 0.655, Rotation2d.fromDegrees(54));
    public static final Pose2d POSE_SOURCE_A_RED = new Pose2d(16.697, 0.655, Rotation2d.fromDegrees(126));
    public static final Pose2d POSE_SOURCE_B_BLUE = new Pose2d(0.851, 7.396, Rotation2d.fromDegrees(306));
    public static final Pose2d POSE_SOURCE_B_RED = new Pose2d(16.697, 0.655, Rotation2d.fromDegrees(126));
    public static final Pose2d POSE_PROCESSOR_BLUE = new Pose2d(5.987, -0.003, Rotation2d.fromDegrees(90));
    public static final Pose2d POSE_PROCESSOR_RED = new Pose2d(11.560, 8.055, Rotation2d.fromDegrees(270));
    public static final Pose2d[][] POSE_CORAL_STANDS_BLUE = {
            {new Pose2d(3.991, 3.448, Rotation2d.fromDegrees(240)), new Pose2d(4.155, 3.164, Rotation2d.fromDegrees(240))},
            {new Pose2d(3.657, 4.189, Rotation2d.fromDegrees(180)), new Pose2d(3.657, 3.861, Rotation2d.fromDegrees(180))},
            {new Pose2d(4.155, 4.887, Rotation2d.fromDegrees(120)), new Pose2d(3.991, 4.603, Rotation2d.fromDegrees(120))},
            {new Pose2d(5.046, 4.603, Rotation2d.fromDegrees(60)), new Pose2d(4.762, 4.887, Rotation2d.fromDegrees(60))},
            {new Pose2d(5.321, 3.861, Rotation2d.fromDegrees(0)), new Pose2d(5.321, 4.189, Rotation2d.fromDegrees(0))},
            {new Pose2d(4.822, 3.164, Rotation2d.fromDegrees(300)), new Pose2d(4.986, 3.448, Rotation2d.fromDegrees(300))}
    };
    public static final Pose2d[][] POSE_CORAL_STANDS_RED = {
            {new Pose2d(13.392, 3.142, Rotation2d.fromDegrees(300)), new Pose2d(13.556, 3.47, Rotation2d.fromDegrees(300))},
            {new Pose2d(13.890, 3.861, Rotation2d.fromDegrees(0)), new Pose2d(13.890, 4.190, Rotation2d.fromDegrees(0))},
            {new Pose2d(13.556, 4.603, Rotation2d.fromDegrees(60)), new Pose2d(13.392, 4.887, Rotation2d.fromDegrees(60))},
            {new Pose2d(12.725, 4.887, Rotation2d.fromDegrees(120)), new Pose2d(12.561, 4.603, Rotation2d.fromDegrees(120))},
            {new Pose2d(12.227, 4.198, Rotation2d.fromDegrees(180)), new Pose2d(12.227, 3.861, Rotation2d.fromDegrees(180))},
            {new Pose2d(12.561, 3.448, Rotation2d.fromDegrees(240)), new Pose2d(12.725, 2.164, Rotation2d.fromDegrees(240))}
    };
    public static final double SOURCE_HEIGHT = 0;
    public static final double CORAL_PODIUM_POLE_HEIGHT = 0;
    public static final double CORAL_LOWER_POLE_HEIGHT = 0;
    public static final double CORAL_MEDIUM_POLE_HEIGHT = 0;
    public static final double CORAL_HIGH_POLE_HEIGHT = 0;
    public static final double PROCESSOR_PLACE_HEIGHT = 0;
    public static final double ROBOT_MAXIMUM_DISTANCE = 2;

    public static boolean isAllianceRed() {
        Optional<DriverStation.Alliance> allianceOptional = DriverStation.getAlliance();
        return allianceOptional.isPresent() && allianceOptional.get() == DriverStation.Alliance.Red;
    }
}
