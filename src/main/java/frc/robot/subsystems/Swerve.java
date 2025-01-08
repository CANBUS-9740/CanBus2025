package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import swervelib.SwerveDrive;
import swervelib.encoders.CANCoderSwerve;
import swervelib.imu.Pigeon2Swerve;
import swervelib.math.SwerveMath;
import swervelib.motors.SparkMaxSwerve;
import swervelib.motors.TalonFXSwerve;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveModulePhysicalCharacteristics;
import swervelib.parser.json.modules.ConversionFactorsJson;
import swervelib.telemetry.SwerveDriveTelemetry;

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
        conversionFactorsJson.drive.diameter = RobotMap.SWERVE_DRIVE_WHEEL_RADIUS * 2;
        conversionFactorsJson.angle.gearRatio = RobotMap.SWERVE_STEER_GEAR_RATIO;
        conversionFactorsJson.angle.factor = 0;
        SwerveModulePhysicalCharacteristics characteristics = new SwerveModulePhysicalCharacteristics(
                conversionFactorsJson,
                RobotMap.SWERVE_WHEEL_FRICTION_COEFFICIENT,
                RobotMap.SWERVE_OPTIMAL_VOLTAGE,
                RobotMap.SWERVE_DRIVE_CURRENT_LIMIT,
                RobotMap.SWERVE_STEER_CURRENT_LIMIT,
                RobotMap.SWERVE_DRIVE_RAMP_RATE,
                RobotMap.SWERVE_STEER_RAMP_RATE,
                RobotMap.SWERVE_DRIVE_FRICTION_VOLTAGE,
                RobotMap.SWERVE_STEER_FRICTION_VOLTAGE,
                RobotMap.SWERVE_STEER_ROTATIONAL_INERTIA,
                RobotMap.ROBOT_MASS_KG
        );

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
                false,
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
                false,
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
                false,
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
                false,
                "BackRight",
                false
        );
        SwerveDriveConfiguration configuration = new SwerveDriveConfiguration(
                new SwerveModuleConfiguration[] {
                        frontLeft, frontRight, backLeft, backRight
                },
                new Pigeon2Swerve(RobotMap.SWERVE_PIGEON_ID),
                true,
                characteristics
        );
        SwerveControllerConfiguration controllerConfiguration = new SwerveControllerConfiguration(
                configuration,
                RobotMap.SWERVE_HEADING_PIDF,
                RobotMap.SWERVE_MAX_SPEED
        );

        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;

        swerveDrive = new SwerveDrive(configuration, controllerConfiguration, RobotMap.SWERVE_MAX_SPEED, new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(false, false, 0);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        swerveDrive.pushOffsetsToEncoders();

        mechanism = new Mechanism2d(50, 50);
        moduleMechanisms = createMechanismDisplay(mechanism);
        SmartDashboard.putData("SwerveMechanism", mechanism);
    }

    public Command drive(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return runEnd(() -> {
                    // Make the robot move
                    swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                                    translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                            Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                            false,
                            false);
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
            SmartDashboard.putNumber("Dergree Module" + i + ": ",modulePositions[i].angle.getDegrees()  );
        }


    }

    private void stop() {
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0));
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