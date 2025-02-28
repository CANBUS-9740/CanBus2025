package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.json.simple.parser.ParseException;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.encoders.CANCoderSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.math.SwerveMath;
import swervelib.motors.SparkMaxSwerve;
import swervelib.motors.TalonFXSwerve;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveModulePhysicalCharacteristics;
import swervelib.parser.json.modules.ConversionFactorsJson;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.IOException;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {

    public final SwerveDrive swerveDrive;

    private final Mechanism2d mechanism;
    private final MechanismLigament2d[] moduleMechanisms;

    public Swerve() {
        ConversionFactorsJson conversionFactorsJson = new ConversionFactorsJson();
        conversionFactorsJson.drive.gearRatio = RobotMap.SWERVE_DRIVE_GEAR_RATIO;
        conversionFactorsJson.drive.factor = 0;
        conversionFactorsJson.drive.diameter = Units.metersToInches(RobotMap.SWERVE_DRIVE_WHEEL_RADIUS * 2);
        conversionFactorsJson.angle.gearRatio = RobotMap.SWERVE_STEER_GEAR_RATIO;
        conversionFactorsJson.angle.factor = 0;

        conversionFactorsJson.drive.calculate();
        conversionFactorsJson.angle.calculate();

        SwerveModulePhysicalCharacteristics characteristics = new SwerveModulePhysicalCharacteristics(
                conversionFactorsJson, RobotMap.SWERVE_DRIVE_RAMP_RATE, RobotMap.SWERVE_STEER_RAMP_RATE);

        SwerveModuleConfiguration frontLeft = new SwerveModuleConfiguration(
                new TalonFXSwerve(RobotMap.SWERVE_DRIVE_FRONT_LEFT_MOTOR_ID, true, DCMotor.getKrakenX60(1)),
                new SparkMaxSwerve(RobotMap.SWERVE_STEER_FRONT_LEFT_MOTOR_ID, false, DCMotor.getNEO(1)),
                conversionFactorsJson,
                new CANCoderSwerve(RobotMap.SWERVE_ENCODER_FRONT_LEFT_ID),
                RobotMap.SWERVE_FRONT_LEFT_ZERO_ANGLE,
                RobotMap.SWERVE_LENGTH / 2,
                RobotMap.SWERVE_WIDTH / 2,
                RobotMap.SWERVE_STEER_PIDF,
                RobotMap.SWERVE_DRIVE_PIDF,
                characteristics,
                false,
                true,
                true,
                "FrontLeft",
                false
        );
        SwerveModuleConfiguration frontRight = new SwerveModuleConfiguration(
                new TalonFXSwerve(RobotMap.SWERVE_DRIVE_FRONT_RIGHT_MOTOR_ID, true, DCMotor.getKrakenX60(1)),
                new SparkMaxSwerve(RobotMap.SWERVE_STEER_FRONT_RIGHT_MOTOR_ID, false, DCMotor.getNEO(1)),
                conversionFactorsJson,
                new CANCoderSwerve(RobotMap.SWERVE_ENCODER_FRONT_RIGHT_ID),
                RobotMap.SWERVE_FRONT_RIGHT_ZERO_ANGLE,
                RobotMap.SWERVE_LENGTH / 2,
                -RobotMap.SWERVE_WIDTH / 2,
                RobotMap.SWERVE_STEER_PIDF,
                RobotMap.SWERVE_DRIVE_PIDF,
                characteristics,
                false,
                true,
                true,
                "FrontRight",
                false
        );
        SwerveModuleConfiguration backLeft = new SwerveModuleConfiguration(
                new TalonFXSwerve(RobotMap.SWERVE_DRIVE_BACK_LEFT_MOTOR_ID, true, DCMotor.getKrakenX60(1)),
                new SparkMaxSwerve(RobotMap.SWERVE_STEER_BACK_LEFT_MOTOR_ID, false, DCMotor.getNEO(1)),
                conversionFactorsJson,
                new CANCoderSwerve(RobotMap.SWERVE_ENCODER_BACK_LEFT_ID),
                RobotMap.SWERVE_BACK_LEFT_ZERO_ANGLE,
                -RobotMap.SWERVE_LENGTH / 2,
                RobotMap.SWERVE_WIDTH / 2,
                RobotMap.SWERVE_STEER_PIDF,
                RobotMap.SWERVE_DRIVE_PIDF,
                characteristics,
                false,
                true,
                true,
                "BackLeft",
                false
        );
        SwerveModuleConfiguration backRight = new SwerveModuleConfiguration(
                new TalonFXSwerve(RobotMap.SWERVE_DRIVE_BACK_RIGHT_MOTOR_ID, true, DCMotor.getKrakenX60(1)),
                new SparkMaxSwerve(RobotMap.SWERVE_STEER_BACK_RIGHT_MOTOR_ID, false, DCMotor.getNEO(1)),
                conversionFactorsJson,
                new CANCoderSwerve(RobotMap.SWERVE_ENCODER_BACK_RIGHT_ID),
                RobotMap.SWERVE_BACK_RIGHT_ZERO_ANGLE,
                -RobotMap.SWERVE_LENGTH / 2,
                -RobotMap.SWERVE_WIDTH / 2,
                RobotMap.SWERVE_STEER_PIDF,
                RobotMap.SWERVE_DRIVE_PIDF,
                characteristics,
                false,
                true,
                true,
                "BackRight",
                false
        );
        SwerveDriveConfiguration configuration = new SwerveDriveConfiguration(
                new SwerveModuleConfiguration[] {
                        frontLeft, frontRight, backLeft, backRight
                },
                new Pigeon2Swerve(RobotMap.SWERVE_PIGEON_ID),
                false,
                characteristics
        );
        SwerveControllerConfiguration controllerConfiguration = new SwerveControllerConfiguration(
                configuration,
                RobotMap.SWERVE_HEADING_PIDF,
                RobotMap.SWERVE_MAX_SPEED
        );

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.POSE;

        swerveDrive = new SwerveDrive(configuration, controllerConfiguration, RobotMap.SWERVE_MAX_SPEED, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        swerveDrive.setHeadingCorrection(true);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(false, false, 0);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        swerveDrive.pushOffsetsToEncoders();
        swerveDrive.setGyroOffset(new Rotation3d(270, 270, 0));

        swerveDrive.resetOdometry(Pose2d.kZero);

        mechanism = new Mechanism2d(50, 50);
        moduleMechanisms = createMechanismDisplay(mechanism);
        SmartDashboard.putData("SwerveMechanism", mechanism);
        pathPlannerSetUp();
    }

    public Field2d getField() {
        return swerveDrive.field;
    }

    public void resetPose(Pose2d pose2d){
        swerveDrive.resetOdometry(pose2d);
    }

    public Command followPathCommand(String pathName) {
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathName);
        } catch (IOException | ParseException e) {
            throw new Error(e);
        }
        return AutoBuilder.followPath(path);
    }


    private void pathPlannerSetUp() {
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new Error(e);
        }

        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speedsRobotRelative, moduleFeedForwards) -> {
                    drive(speedsRobotRelative);
                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(10, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                     // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public double getDistanceToMeters(Pose2d robotPose, Pose2d pos) {
        return Math.sqrt(Math.pow(robotPose.getX() - pos.getX(), 2) + Math.pow(robotPose.getY() - pos.getY(), 2));
    }

    public Command drive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return runEnd(() -> {
                    Translation2d translation2d = SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8);
                    double rotation = Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity();
                    drive(new ChassisSpeeds(translation2d.getX(), translation2d.getY(), rotation));
                },
                this::stop);
    }

    public Command fieldDrive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return runEnd(() -> {
                    Translation2d translation2d = SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8);
                    double rotation = Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity();
                    fieldDrive(new ChassisSpeeds(translation2d.getX(), translation2d.getY(), rotation));
                },
                this::stop);
    }

    public Command centerModules() {
        return run(() -> Arrays.asList(swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }

    @Override
    public void periodic() {
        SwerveModulePosition[] modulePositions = swerveDrive.getModulePositions();
        for (int i = 0; i < modulePositions.length; i++) {
            moduleMechanisms[i].setAngle(modulePositions[i].angle.getDegrees() + 90);
            SmartDashboard.putNumber("ModuleHeading " + i, modulePositions[i].angle.getDegrees());
        }
    }

    public void drive(ChassisSpeeds speeds) {
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
            stop();
        } else {
            swerveDrive.drive(speeds, Translation2d.kZero);
        }
    }

    private void fieldDrive(ChassisSpeeds speeds) {
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
            stop();
        } else {
            swerveDrive.driveFieldOriented(speeds, Translation2d.kZero);
        }
    }

    private void stop() {
        for (SwerveModule module : swerveDrive.getModules()) {
            module.getDriveMotor().set(0);
            module.getAngleMotor().set(0);
        }
    }

    private MechanismLigament2d[] createMechanismDisplay(Mechanism2d mechanism) {
        final double BOTTOM = 5;
        final double TOP = 45;
        final double BEAM_LENGTH = 40;
        final double BEAM_WIDTH = 2;
        final double WHEEL_DIR_LENGTH = 5;
        final double WHEEL_DIR_WIDTH = 10;
        final Color8Bit BEAM_COLOR = new Color8Bit(0, 254, 52);
        final Color8Bit WHEEL_DIR_COLOR = new Color8Bit(255, 0, 0);

        MechanismRoot2d driveBaseMechanismBottomLeft = mechanism.getRoot("drivebase-bottomleft", BOTTOM, BOTTOM);
        MechanismRoot2d driveBaseMechanismTopRight = mechanism.getRoot("drivebase-topright", TOP, TOP);
        MechanismRoot2d driveBaseMechanismBottomRight = mechanism.getRoot("drivebase-bottomright", TOP, BOTTOM);
        MechanismRoot2d driveBaseMechanismTopLeft = mechanism.getRoot("drivebase-topleft", BOTTOM, TOP);

        driveBaseMechanismBottomLeft.append(new MechanismLigament2d("bottom", BEAM_LENGTH, 0, BEAM_WIDTH, BEAM_COLOR));
        driveBaseMechanismBottomLeft.append(new MechanismLigament2d("left", BEAM_LENGTH, 90, BEAM_WIDTH, BEAM_COLOR));
        driveBaseMechanismTopRight.append(new MechanismLigament2d("top", BEAM_LENGTH, 180, BEAM_WIDTH, BEAM_COLOR));
        driveBaseMechanismTopRight.append(new MechanismLigament2d("right", BEAM_LENGTH, 270, BEAM_WIDTH, BEAM_COLOR));

        MechanismLigament2d mechanismBottomLeft = driveBaseMechanismBottomLeft.append(new MechanismLigament2d("module-bottomleft", WHEEL_DIR_LENGTH, 90, WHEEL_DIR_WIDTH, WHEEL_DIR_COLOR));
        MechanismLigament2d mechanismBottomRight = driveBaseMechanismBottomRight.append(new MechanismLigament2d("module-bottomright", WHEEL_DIR_LENGTH, 90, WHEEL_DIR_WIDTH, WHEEL_DIR_COLOR));
        MechanismLigament2d mechanismTopLeft = driveBaseMechanismTopLeft.append(new MechanismLigament2d("module-topleft", WHEEL_DIR_LENGTH, 90, WHEEL_DIR_WIDTH, WHEEL_DIR_COLOR));
        MechanismLigament2d mechanismTopRight = driveBaseMechanismTopRight.append(new MechanismLigament2d("module-topright", WHEEL_DIR_LENGTH, 90, WHEEL_DIR_WIDTH, WHEEL_DIR_COLOR));

        return new MechanismLigament2d[] {
                mechanismTopLeft,
                mechanismTopRight,
                mechanismBottomLeft,
                mechanismBottomRight
        };
    }
}