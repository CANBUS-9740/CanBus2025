package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmJointControlCommand;
import frc.robot.commands.ClawGripperIntake;
import frc.robot.commands.ClawGripperOuttake;
import frc.robot.commands.ClawGripperOuttakeSlow;
import frc.robot.subsystems.*;
import org.opencv.core.Core;
import org.opencv.core.Mat;

import java.util.Locale;
import java.util.Optional;
import java.util.Set;

public class Robot extends TimedRobot {

    private GameField gameField;
    private Swerve swerve;
    private Command auto;
    private LimeLight limeLight;

    private ClawGripperSystem clawGripperSystem;
    private ArmJointSystem armJointSystem;

    private ArmJointControlCommand armJointControlCommand;

    private CommandXboxController controllerXbox;
    private CommandXboxController driverXbox;
    private SendableChooser<Command> autoChooser;

    private UsbCamera usbCamera;
    private CvSink cvSink;
    private CvSource outputStream;
    private Mat orgMat;
    private Mat dstMat;

    @Override
    public void robotInit() {
        gameField = new GameField();
        swerve = new Swerve();
        clawGripperSystem = new ClawGripperSystem();
        armJointSystem = new ArmJointSystem();
        limeLight = new LimeLight(RobotMap.APRIL_TAG_LIMELIGHT_NAME);
        armJointControlCommand = new ArmJointControlCommand(armJointSystem);

        usbCamera = CameraServer.startAutomaticCapture();
        cvSink = CameraServer.getVideo();
        outputStream = CameraServer.putVideo("OutputStream", 640, 480);
        orgMat = new Mat();
        dstMat = new Mat();

        armJointSystem.setDefaultCommand(armJointControlCommand);

        driverXbox = new CommandXboxController(0);
        driverXbox.leftBumper().onTrue(new InstantCommand(() -> swerve.resetPose(new Pose2d(0, 0, new Rotation2d()))));
        controllerXbox = new CommandXboxController(1);

        swerve.setDefaultCommand(createSwerveDrive());

        Command
                goAndCollectFromClosestSource = Commands.defer(() -> {
            Optional<GameField.SelectedSourceStand> optional = getClosestSource();
            if (optional.isEmpty()) {
                return Commands.none();
            }

            GameField.SelectedSourceStand stand = optional.get();

            return goToSourceAndCollectTeleop(stand.stand, GameField.SourceStandSide.CENTER);
        }, Set.of(swerve, clawGripperSystem));

//        controllerXbox.y().onTrue(goToReefAndPlaceDefer(GameField.ReefStandSide.RIGHT, ReefHeight.SECOND_STAGE));
//        controllerXbox.b().onTrue(goToReefAndPlaceDefer(GameField.ReefStandSide.RIGHT, ReefHeight.FIRST_STAGE));
//        controllerXbox.a().onTrue(goToReefAndPlaceDefer(GameField.ReefStandSide.RIGHT, ReefHeight.PODIUM));
//        controllerXbox.pov(0).onTrue(goToReefAndPlaceDefer(GameField.ReefStandSide.LEFT, ReefHeight.SECOND_STAGE));
//        controllerXbox.pov(270).onTrue(goToReefAndPlaceDefer(GameField.ReefStandSide.LEFT, ReefHeight.FIRST_STAGE));
//        controllerXbox.pov(180).onTrue(goToReefAndPlaceDefer(GameField.ReefStandSide.LEFT, ReefHeight.PODIUM));
        controllerXbox.back().onTrue(Commands.runOnce(() -> {
            armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE);
        }, swerve, clawGripperSystem));

        controllerXbox.x().onTrue(goAndCollectFromClosestSource);
        controllerXbox.y().onTrue(moveArmToAngle(RobotMap.ARM_JOINT_ANGLE_SECOND));
        controllerXbox.b().onTrue(moveArmToAngle(RobotMap.ARM_JOINT_ANGLE_FIRST));
        controllerXbox.a().onTrue(moveArmToAngle(RobotMap.ARM_JOINT_ANGLE_PODIUM));
        controllerXbox.rightBumper().onTrue(
                new SequentialCommandGroup(
                        new ClawGripperOuttake(clawGripperSystem),
                        moveArmToAngle(RobotMap.ARM_JOINT_DEFAULT_ANGLE)
                )
        );
        controllerXbox.leftBumper().onTrue(
                new SequentialCommandGroup(
                        new ClawGripperOuttakeSlow(clawGripperSystem),
                        moveArmToAngle(RobotMap.ARM_JOINT_DEFAULT_ANGLE)
                )
        );
        controllerXbox.pov(0).onTrue(
                new ClawGripperOuttakeSlow(clawGripperSystem)
        );
        controllerXbox.pov(180).onTrue(
                new SequentialCommandGroup(
                        moveArmToAngle(RobotMap.ARM_JOINT_ANGLE_SOURCE),
                        new ClawGripperIntake(clawGripperSystem),
                        moveArmToAngle(RobotMap.ARM_JOINT_DEFAULT_ANGLE)
                )
        );
        //driverXbox.x().onTrue(collectFromSource());
        driverXbox.rightBumper().onTrue(Commands.runOnce(() -> {
            armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE);
        }, swerve, clawGripperSystem));

        // we might need to change it to gripper outtake with no automation that's for giving
        // the driver time to adjust to the reef

        FollowPathCommand.warmupCommand().schedule();
        autoChooser = new SendableChooser<>();

        autoChooser.setDefaultOption("default", Commands.none());
        autoChooser.addOption("drive", new SequentialCommandGroup(
                swerve.drive(
                        () -> -0.11,
                        () -> 0,
                        () -> 0,
                        false
                ).withTimeout(1)
        ));
        autoChooser.addOption("drive and output", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        swerve.drive(
                                () -> -0.11,
                                () -> 0,
                                () -> 0,
                                false
                        ).withTimeout(3),
                        moveArmToAngle(RobotMap.ARM_JOINT_ANGLE_PODIUM)
                ),
                new ClawGripperOuttakeSlow(clawGripperSystem).withTimeout(1),
                Commands.waitSeconds(3),
                swerve.drive(
                        () -> 0.11,
                        () -> 0,
                        () -> 0,
                        false
                ).withTimeout(1)
        ));
        autoChooser.addOption("Go To Source Left And Collect", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        goToSource(GameField.SourceStand.LEFT, GameField.SourceStandSide.CENTER),
                        moveArmToAngle(RobotMap.ARM_JOINT_ANGLE_SOURCE)
                ),
                new ClawGripperIntake(clawGripperSystem)
        ));
        autoChooser.addOption("Go To Reef 3_L3, Source Left, Reef 2_L2",
                new SequentialCommandGroup(
                        goToReefAndPlaceAuto(GameField.ReefStand.STAND_3, GameField.ReefStandSide.RIGHT, ReefHeight.SECOND_STAGE),
                        goToSourceAndCollectAuto(GameField.SourceStand.LEFT, GameField.SourceStandSide.CENTER),
                        goToReefAndPlaceAuto(GameField.ReefStand.STAND_2, GameField.ReefStandSide.RIGHT, ReefHeight.FIRST_STAGE)
                ));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        if (cvSink.grabFrame(orgMat) > 0) {
            double armAngle = armJointSystem.getRawPositionDegrees();
            ;
            if (armAngle > 180) {
                Core.flip(orgMat, dstMat, -1);
            } else {
                Core.copyTo(orgMat, dstMat, orgMat);
            }
            outputStream.putFrame(dstMat);
        }

        Optional<GameField.SelectedReefStand> standOptional = getBestStand();
        if (standOptional.isPresent()) {
            GameField.SelectedReefStand stand = standOptional.get();
            swerve.getField().getObject("BestStand").setPose(stand.pose);
            SmartDashboard.putString("BestStand", String.format(Locale.ENGLISH, "%s.%s", stand.stand.name(), stand.side.name()));
            SmartDashboard.putBoolean("HasBestStand", true);
        } else {
            swerve.getField().getObject("BestStand").setPoses();
            SmartDashboard.putString("BestStand", "");
            SmartDashboard.putBoolean("HasBestStand", false);
        }

        standOptional = getClosestStand();
        if (standOptional.isPresent()) {
            GameField.SelectedReefStand stand = standOptional.get();
            swerve.getField().getObject("ClosestStand").setPose(stand.pose);
            SmartDashboard.putString("ClosestStand", String.format(Locale.ENGLISH, "%s.%s", stand.stand.name(), stand.side.name()));
        } else {
            swerve.getField().getObject("ClosestStand").setPoses();
            SmartDashboard.putString("ClosestStand", "");
        }

        Optional<GameField.SelectedSourceStand> sourceOptional = getClosestSource();
        if (sourceOptional.isPresent()) {
            GameField.SelectedSourceStand stand = sourceOptional.get();
            SmartDashboard.putString("ClosestSource", String.format(Locale.ENGLISH, "%s.%s", stand.stand.name(), stand.side.name()));
            swerve.getField().getObject("ClosestSource").setPose(stand.pose);
        } else {
            SmartDashboard.putString("ClosestSource", "");
            swerve.getField().getObject("ClosestSource").setPoses();
        }

        Optional<LimelightHelpers.PoseEstimate> poseEstimateOptional = limeLight.getPose();
        if (poseEstimateOptional.isPresent()) {
            LimelightHelpers.PoseEstimate poseEstimate = poseEstimateOptional.get();
            swerve.addVisionMeasurement(poseEstimate);
        }
    }

    @Override
    public void simulationInit() {

    }

    @Override
    public void simulationPeriodic() {

    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void teleopExit() {

    }

    @Override
    public void autonomousInit() {
        auto = autoChooser.getSelected();
        if (auto != null) {
            auto.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        if (auto != null) {
            auto.cancel();
            auto = null;
        }
    }

    @Override
    public void testInit() {

    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }

    private Optional<GameField.SelectedReefStand> getBestStand() {
        Pose2d pose = swerve.getPose();
        return gameField.findBestReefStandTo(pose, true);
    }

    private Optional<GameField.SelectedReefStand> getClosestStand() {
        Pose2d pose = swerve.getPose();
        return gameField.findBestReefStandTo(pose, false);
    }

    private Optional<GameField.SelectedSourceStand> getClosestSource() {
        Pose2d robotPose = swerve.getPose();
        return gameField.getClosestSourceTo(robotPose);
    }

    private Command goToReefAndPlaceDefer(GameField.ReefStandSide side, ReefHeight height) {
        return Commands.defer(() -> {
            Optional<GameField.SelectedReefStand> optional = getClosestStand();
            if (optional.isEmpty()) {
                return Commands.none();
            }

            GameField.SelectedReefStand stand = optional.get();
            return goToReefAndPlaceTeleop(stand.stand, side, height);
        }, Set.of(swerve, clawGripperSystem));
    }

    private Command goToReefAndPlaceTeleop(GameField.ReefStand stand, GameField.ReefStandSide side, ReefHeight height) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        goToReef(stand, side, height),
                        moveArmToAngle(getArmAngleForReef(height))
                ),
                new ParallelDeadlineGroup(
                        new SequentialCommandGroup(
                                Commands.waitUntil(() -> controllerXbox.x().getAsBoolean()),
                                new ClawGripperOuttake(clawGripperSystem)
                        ),
                        createSwerveDrive()
                ),
                Commands.runOnce(() -> armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE))
        );
    }

    private Command goToSourceAndCollectTeleop(GameField.SourceStand stand, GameField.SourceStandSide side) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        goToSource(stand, side),
                        moveArmToAngle(RobotMap.ARM_JOINT_ANGLE_SOURCE)
                ),
                new ParallelDeadlineGroup(
                        new ClawGripperIntake(clawGripperSystem),
                        createSwerveDrive()
                ),
                Commands.runOnce(() -> armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE))
        );

    }

    private Command goToReefAndPlaceAuto(GameField.ReefStand stand, GameField.ReefStandSide side, ReefHeight height) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        goToReef(stand, side, height),
                        moveArmToAngle(getArmAngleForReef(height))
                ),
                new ClawGripperOuttake(clawGripperSystem),
                Commands.runOnce(() -> armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE))
        );
    }

    private Command goToSourceAndCollectAuto(GameField.SourceStand stand, GameField.SourceStandSide side) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        goToSource(stand, side),
                        moveArmToAngle(RobotMap.ARM_JOINT_ANGLE_SOURCE)
                ),
                new ClawGripperIntake(clawGripperSystem),
                Commands.runOnce(() -> armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE))
        );
    }

    private double getArmAngleForReef(ReefHeight reefHeight) {
        double armAngle;
        switch (reefHeight) {
            case PODIUM:
                armAngle = RobotMap.ARM_JOINT_ANGLE_PODIUM;
                break;
            case FIRST_STAGE:
                armAngle = RobotMap.ARM_JOINT_ANGLE_FIRST;
                break;
            case SECOND_STAGE:
                armAngle = RobotMap.ARM_JOINT_ANGLE_SECOND;
                break;
            case THIRD_STAGE:
                armAngle = RobotMap.ARM_JOINT_ANGLE_THIRD;
                break;
            default:
                return RobotMap.ARM_JOINT_DEFAULT_ANGLE;
        }

        return armAngle;
    }

    private Command moveArmToAngle(double armAngle) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> armJointControlCommand.setTargetPosition(armAngle)),
                Commands.waitUntil(() -> armJointControlCommand.isAtTargetPosition())
        );
    }

    private Command collectFromSource() {
        return new SequentialCommandGroup(
                moveArmToAngle(RobotMap.ARM_JOINT_ANGLE_SOURCE),
                new ClawGripperIntake(clawGripperSystem)
        );
    }

    private Command goToSource(GameField.SourceStand stand, GameField.SourceStandSide side) {
        Pose2d pose = gameField.getPoseForSource(stand, side);
        double newRotation = (180 + pose.getRotation().getDegrees()) % 360;
        return goToPose(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(newRotation)));
    }

    private Command goToReef(GameField.ReefStand stand, GameField.ReefStandSide side, ReefHeight height) {
        Pose2d pose = gameField.getPoseForReefStand(stand, side);
        if (height == ReefHeight.PODIUM) {
            double newRotation = (180 + pose.getRotation().getDegrees()) % 360;
            pose = new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(newRotation));
        }

        return goToPose(pose);
    }

    private Command goToPose(Pose2d pose) {
        return new SequentialCommandGroup(
                Commands.runOnce(() -> {
                    System.out.printf("Going to Pose: %s\n", pose.toString());
                    swerve.getField().getObject("Target").setPose(pose);
                }),
                AutoBuilder.pathfindToPose(pose, RobotMap.PATHFIND_CONSTRAINTS),
                Commands.runOnce(() -> System.out.println("Done going to Pose"))
        );
    }

    private Command createSwerveDrive() {
        return swerve.drive(
                () -> -MathUtil.applyDeadband(Math.pow(driverXbox.getRightY(), 3), 0.05),
                () -> -MathUtil.applyDeadband(Math.pow(driverXbox.getRightX(), 3), 0.05),
                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), 0.15),
                true
        );
    }
}
