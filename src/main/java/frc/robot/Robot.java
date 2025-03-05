package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmJointSystem;
import frc.robot.subsystems.ClawGripperSystem;
import frc.robot.subsystems.Swerve;

import java.util.Locale;
import java.util.Optional;
import java.util.Set;

public class Robot extends TimedRobot {

    private GameField gameField;
    private Swerve swerve;
    private Command auto;

    private ClawGripperSystem clawGripperSystem;
    private ArmJointSystem armJointSystem;

    private ArmJointControlCommand armJointControlCommand;

    private XboxController controllerXbox;
    private XboxController driverXbox;
    private SendableChooser<Command> autoChooser;

    @Override
    public void robotInit() {
        gameField = new GameField();
        swerve = new Swerve();
        clawGripperSystem = new ClawGripperSystem();

        driverXbox = new XboxController(0);
        controllerXbox = new XboxController(1);

        swerve.setDefaultCommand(swerve.fieldDrive(
                () -> MathUtil.applyDeadband(Math.pow(driverXbox.getRightY(), 3), 0.05),
                () -> MathUtil.applyDeadband(Math.pow(driverXbox.getRightX(), 3), 0.05),
                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), 0.15)
        ));

        new JoystickButton(controllerXbox, XboxController.Button.kB.value).whileTrue(
                new ClawGripperOuttake(clawGripperSystem)
        );

        new JoystickButton(controllerXbox, XboxController.Button.kX.value).whileTrue(
                new ClawGripperIntake(clawGripperSystem)
        );

        new POVButton(controllerXbox, 270).onTrue(
                outtakeReef(CoralReef.FIRST_STAGE)
        );

        new POVButton(controllerXbox, 180).onTrue(
                outtakeReef(CoralReef.SECOND_STAGE)
        );

        new POVButton(controllerXbox, 0).onTrue(
                outtakeReef(CoralReef.THIRD_STAGE)
        );

        FollowPathCommand.warmupCommand().schedule();
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("drive", swerve.drive(
                ()-> 0.1,
                ()-> 0,
                ()-> 0
        ).withTimeout(2));

        autoChooser.setDefaultOption("default", Commands.none());
        autoChooser.addOption("drive and output", new SequentialCommandGroup(
                swerve.drive(
                        ()-> 0.272,
                        ()-> 0,
                        ()-> 0
                ).withTimeout(5),
                new ClawGripperOuttake(clawGripperSystem).withTimeout(2)
        ));
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

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

    private boolean isCommandNotValid(double armAngle, double distanceToTarget) {
        return armAngle < RobotMap.ARM_JOINT_MINIMUM_ANGLE ||
                armAngle > RobotMap.ARM_JOINT_MAXIMUM_ANGLE ||
                distanceToTarget > RobotMap.ROBOT_MAXIMUM_DISTANCE;
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

    private Command placeCoralOnReefCommand(CoralReef coralReef) {
        double reefPoleHeight;
        switch (coralReef) {
            case PODIUM:
                reefPoleHeight = RobotMap.CORAL_PODIUM_POLE_HEIGHT;
                break;
            case FIRST_STAGE:
                reefPoleHeight = RobotMap.CORAL_LOWER_POLE_HEIGHT;
                break;
            case SECOND_STAGE:
                reefPoleHeight = RobotMap.CORAL_MEDIUM_POLE_HEIGHT;
                break;
            case THIRD_STAGE:
                reefPoleHeight = RobotMap.CORAL_HIGH_POLE_HEIGHT;
                break;
            default:
                reefPoleHeight = 0;
                break;
        }

        return Commands.defer(() -> {
            Optional<GameField.SelectedReefStand> standOptional = getBestStand();
            if (standOptional.isEmpty()) {
                return Commands.none();
            }

            Pose2d stand = standOptional.get().pose;
            Pose2d robotPose = swerve.getPose();

            double distance = swerve.getDistanceToMeters(robotPose, stand);
            double angle = armJointSystem.calculateAngleForTarget(distance, reefPoleHeight);

            if (isCommandNotValid(angle, distance)) {
                return Commands.none();
            }

            return new SequentialCommandGroup(
                    createCommandGroupSimple(angle),
                    new ClawGripperOuttake(clawGripperSystem),
                    Commands.runOnce(()-> armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE))
            );
        }, Set.of(clawGripperSystem));
    }

    private Command placeInClosestSource() {
        return Commands.defer(() -> {
            Pose2d robotPose = swerve.getPose();
            Optional<GameField.SelectedSourceStand> standOptional = getClosestSource();
            if (standOptional.isEmpty()) {
                return Commands.none();
            }

            Pose2d sourcePose = standOptional.get().pose;
            double targetsDistance = swerve.getDistanceToMeters(robotPose, sourcePose);
            double angle = armJointSystem.calculateAngleForTarget(targetsDistance, RobotMap.SOURCE_HEIGHT);

            if (isCommandNotValid(angle, targetsDistance)) {
                return Commands.none();
            }

            return new SequentialCommandGroup(
                    createCommandGroupSimple(angle),
                    new ClawGripperIntake(clawGripperSystem),
                    Commands.runOnce(()-> armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE))
            );
        }, Set.of(clawGripperSystem));
    }

    private Command placeCoralOnReefCommandSimple(CoralReef coralReef) {
        double armAngle;
        switch (coralReef) {
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
                return null;
        }

        return createCommandGroupSimple(armAngle);
    }

    private SequentialCommandGroup outtakeReef(CoralReef coralReef){
        return new SequentialCommandGroup(
                placeCoralOnReefCommandSimple(coralReef),
                new ClawGripperOuttake(clawGripperSystem),
                Commands.runOnce(()-> armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE))
        );
    }

    private Command collectFromSourceCommandSimple() {
        return createCommandGroupSimple(RobotMap.ARM_JOINT_ANGLE_SOURCE);
    }

    private Command createCommandGroupSimple(double armAngle) {
        return new SequentialCommandGroup(
                Commands.runOnce(()-> armJointControlCommand.setTargetPosition(armAngle)),
                Commands.waitUntil(()-> armJointControlCommand.isAtTargetPosition())
        );
    }
}