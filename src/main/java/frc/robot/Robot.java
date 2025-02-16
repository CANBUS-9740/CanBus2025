package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmJointControlCommand;
import frc.robot.commands.ArmTelescopicHold;
import frc.robot.commands.ArmTelescopicMoveToLength;
import frc.robot.commands.ArmTelescopicReset;
import frc.robot.commands.ClawGripperIntake;
import frc.robot.commands.ClawGripperOuttake;
import frc.robot.commands.HoldItemInClawGripper;
import frc.robot.commands.MoveClawJointToPosition;
import frc.robot.subsystems.ArmJointSystem;
import frc.robot.subsystems.ArmTelescopicSystem;
import frc.robot.subsystems.ClawGripperSystem;
import frc.robot.subsystems.ClawJointSystem;
import frc.robot.subsystems.Swerve;

import java.util.Locale;
import java.util.Optional;
import java.util.Set;

public class Robot extends TimedRobot {

    private GameField gameField;
    private Swerve swerve;
    private Command auto;

    private ClawGripperSystem clawGripperSystem;
    private ClawJointSystem clawJointSystem;
    private ArmJointSystem armJointSystem;
    private ArmTelescopicSystem armTelescopicSystem;

    private ArmJointControlCommand armJointControlCommand;

    private XboxController xbox;
    private SendableChooser<Command> autoChooser;

    @Override
    public void robotInit() {
        gameField = new GameField();
        swerve = new Swerve();
        clawGripperSystem = new ClawGripperSystem();
        armJointSystem = new ArmJointSystem();
        clawJointSystem = new ClawJointSystem();
        armTelescopicSystem = new ArmTelescopicSystem();

        xbox = new XboxController(0);

        armJointControlCommand = new ArmJointControlCommand(armJointSystem);
        armJointSystem.setDefaultCommand(armJointControlCommand);

        armTelescopicSystem.setDefaultCommand(
                new SequentialCommandGroup(
                        new ArmTelescopicReset(armTelescopicSystem),
                        new ArmTelescopicHold(armTelescopicSystem)
                )
        );
        clawGripperSystem.setDefaultCommand(
                Commands.defer(() -> {
                    if (clawGripperSystem.hasItem()) {
                        return new HoldItemInClawGripper(clawGripperSystem);
                    }
                    return Commands.idle(clawGripperSystem);
                }, Set.of(clawGripperSystem))
        );

        Command collectFromSource = Commands.defer(() -> {
            Pose2d robotPose = swerve.getPose();
            Optional<GameField.SelectedSourceStand> standOptional = getClosestSource();
            if (standOptional.isEmpty()) {
                return Commands.none();
            }

            Pose2d sourcePose = standOptional.get().pose;
            double targetsDistance = swerve.getDistanceToMeters(robotPose, sourcePose);
            double length = armTelescopicSystem.calculateLengthForTarget(targetsDistance, RobotMap.SOURCE_HEIGHT);
            double angle = armJointSystem.calculateAngleForTarget(targetsDistance, RobotMap.SOURCE_HEIGHT);

            if (isCommandNotValid(length, angle, targetsDistance)) {
                return Commands.none();
            }

            return new SequentialCommandGroup(
                    createCommandGroupSimple(length, angle, RobotMap.CLAWJOINT_SOURCE_ANGLE),
                    new ClawGripperIntake(clawGripperSystem)
            );
        }, Set.of(armTelescopicSystem, clawJointSystem, clawGripperSystem));

        SequentialCommandGroup collectFromFloor = new SequentialCommandGroup(
                createCommandGroupSimple(RobotMap.ARM_LENGTH_FLOOR, RobotMap.ARM_JOINT_FLOOR_ANGLE, RobotMap.CLAWJOINT_FLOOR_ANGLE),
                new ClawGripperIntake(clawGripperSystem)
        );

        Command placeOnReefPodium = placeCoralOnReefCommand(CoralReef.PODIUM);
        Command placeOnReefFirstStage = placeCoralOnReefCommand(CoralReef.FIRST_STAGE);
        Command placeOnReefSecondStage = placeCoralOnReefCommand(CoralReef.SECOND_STAGE);
        Command placeOnReefThirdStage = placeCoralOnReefCommand(CoralReef.THIRD_STAGE);

        Command placeInProcessor = Commands.defer(() -> {
            Pose2d robotPose = swerve.getPose();
            Pose2d processorPose = gameField.getPoseToProcessor();
            double distance = swerve.getDistanceToMeters(robotPose, processorPose);

            double length = armTelescopicSystem.calculateLengthForTarget(distance, RobotMap.PROCESSOR_PLACE_HEIGHT);
            double angle = armJointSystem.calculateAngleForTarget(distance, RobotMap.PROCESSOR_PLACE_HEIGHT);

            if (isCommandNotValid(length, angle, distance)) {
                return Commands.none();
            }

            return new SequentialCommandGroup(
                    createCommandGroupSimple(length, angle, RobotMap.CLAWJOINT_PROCESSOR_ANGLE),
                    new ClawGripperOuttake(clawGripperSystem)
            );
        }, Set.of(armTelescopicSystem, clawJointSystem, clawGripperSystem));

        ParallelCommandGroup underCage = new ParallelCommandGroup(
                new ArmTelescopicReset(armTelescopicSystem),
                Commands.runOnce(()->  armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_UNDER_CAGE_ANGLE)),
                new MoveClawJointToPosition(clawJointSystem, RobotMap.CLAWJOINT_UNDER_CAGE_ANGLE)
        );

        ParallelCommandGroup hang = new ParallelCommandGroup(
                new ArmTelescopicReset(armTelescopicSystem),
                Commands.runOnce(()->  armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_FLOOR_ANGLE))
        );

        SequentialCommandGroup reefLowerAlgae = new SequentialCommandGroup(
                createCommandGroupSimple(RobotMap.ARM_TELESCOPIC_LOWER_REEF_ALGAE_LENGTH, RobotMap.ARM_JOINT_LOWER_REEF_ALGAE_ANGLE, RobotMap.CLAWJOINT_LOWER_REEF_ALGAE_ANGLE),
                new ClawGripperIntake(clawGripperSystem)
        );

        SequentialCommandGroup reefHighAlgae = new SequentialCommandGroup(
                createCommandGroupSimple(RobotMap.ARM_TELESCOPIC_HIGH_REEF_ALGAE_LENGTH, RobotMap.ARM_JOINT_HIGH_REEF_ALGAE_ANGLE, RobotMap.CLAWJOINT_HIGH_REEF_ALGAE_ANGLE),
                new ClawGripperIntake(clawGripperSystem)
        );

        Command collectFromSourceCommandSimple =
                createCommandGroupSimple(RobotMap.CALCULATION_COLLECT_FROM_SOURCE, RobotMap.ARM_JOINT_ANGLE_SOURCE, RobotMap.CLAWJOINT_SOURCE_ANGLE);
        Command placeInProcessorCommandSimple =
                createCommandGroupSimple(RobotMap.CALCULATION_PLACE_IN_PROCESSOR, RobotMap.ANGLE_PROCESSOR, RobotMap.CLAWJOINT_PROCESSOR_ANGLE);

        new POVButton(xbox, 180).onTrue(placeCoralOnReefCommandSimple(CoralReef.PODIUM));
        new POVButton(xbox, 270).onTrue(placeCoralOnReefCommandSimple(CoralReef.FIRST_STAGE));
        new POVButton(xbox, 90).onTrue(placeCoralOnReefCommandSimple(CoralReef.SECOND_STAGE));
        new POVButton(xbox, 0).onTrue(placeCoralOnReefCommandSimple(CoralReef.THIRD_STAGE));

        new JoystickButton(xbox, XboxController.Button.kRightBumper.value).onTrue(placeInProcessorCommandSimple);
        new JoystickButton(xbox, XboxController.Button.kLeftBumper.value).onTrue(collectFromSourceCommandSimple);
        new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new ClawGripperOuttake(clawGripperSystem));
        new JoystickButton(xbox, XboxController.Button.kX.value).onTrue(new ClawGripperIntake(clawGripperSystem));
        new JoystickButton(xbox, XboxController.Button.kA.value).onTrue(reefHighAlgae);
        new JoystickButton(xbox, XboxController.Button.kY.value).onTrue(placeInProcessor);
        new Trigger(() -> xbox.getRightTriggerAxis() > 0.5).onTrue(reefLowerAlgae);
        new Trigger(() -> xbox.getLeftTriggerAxis() > 0.5).onTrue(hang);

        FollowPathCommand.warmupCommand().schedule();
        autoChooser = AutoBuilder.buildAutoChooser();
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
        swerve.fieldDrive(
                () -> -MathUtil.applyDeadband(Math.pow(xbox.getRightY(), 3), 0.05),
                () -> MathUtil.applyDeadband(Math.pow(xbox.getRightX(), 3), 0.05),
                () -> MathUtil.applyDeadband(xbox.getLeftX(), 0.15)
        ).schedule();
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

    private boolean isCommandNotValid(double length, double angle, double distance) {
        return length > RobotMap.ARM_TELESCOPIC_MAXIMUM_LENGTH ||
                length < RobotMap.ARM_TELESCOPIC_MINIMUM_LENGTH ||
                angle < RobotMap.ARM_JOINT_MINIMUM_ANGLE ||
                angle > RobotMap.ARM_JOINT_MAXIMUM_ANGLE ||
                distance > RobotMap.ROBOT_MAXIMUM_DISTANCE;
    }

    private Optional<GameField.SelectedReefStand> getBestStand() {
        Pose2d pose = swerve.getPose();
        return gameField.findBestReefStandTo(pose, false);
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
        double crawJointAngle;
        switch (coralReef) {
            case PODIUM:
                reefPoleHeight = RobotMap.CORAL_PODIUM_POLE_HEIGHT;
                crawJointAngle = RobotMap.CLAWJOINT_CORAL_PODIUM_POLE_ANGLE;
                break;
            case FIRST_STAGE:
                reefPoleHeight = RobotMap.CORAL_LOWER_POLE_HEIGHT;
                crawJointAngle = RobotMap.CLAWJOINT_CORAL_FIRST_POLE_ANGLE;
                break;
            case SECOND_STAGE:
                reefPoleHeight = RobotMap.CORAL_MEDIUM_POLE_HEIGHT;
                crawJointAngle = RobotMap.CLAWJOINT_CORAL_SECOND_POLE_ANGLE;
                break;
            case THIRD_STAGE:
                reefPoleHeight = RobotMap.CORAL_HIGH_POLE_HEIGHT;
                crawJointAngle = RobotMap.CLAWJOINT_CORAL_THIRD_POLE_ANGLE;
                break;
            default:
                reefPoleHeight = 0;
                crawJointAngle = 0;
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
            double length = armTelescopicSystem.calculateLengthForTarget(distance, reefPoleHeight);
            double angle = armJointSystem.calculateAngleForTarget(distance, reefPoleHeight);

            if (isCommandNotValid(length, angle, distance)) {
                return Commands.none();
            }

            return new SequentialCommandGroup(
                    createCommandGroupSimple(length, angle, crawJointAngle),
                    new ClawGripperOuttake(clawGripperSystem)
            );
        }, Set.of(armTelescopicSystem, clawJointSystem, clawGripperSystem));
    }

    private Command placeCoralOnReefCommandSimple(CoralReef coralReef) {
        double armLength;
        double armAngle;
        double clawAngle;
        switch (coralReef) {
            case PODIUM:
                armLength = RobotMap.ARM_LENGTH_PODIUM;
                armAngle = RobotMap.ARM_JOINT_ANGLE_PODIUM;
                clawAngle = RobotMap.CLAWJOINT_CORAL_PODIUM_POLE_ANGLE;
                break;
            case FIRST_STAGE:
                armLength = RobotMap.ARM_LENGTH_FIRST;
                armAngle = RobotMap.ARM_JOINT_ANGLE_FIRST;
                clawAngle = RobotMap.CLAWJOINT_CORAL_FIRST_POLE_ANGLE;
                break;
            case SECOND_STAGE:
                armLength = RobotMap.ARM_LENGTH_SECOND;
                armAngle = RobotMap.ARM_JOINT_ANGLE_SECOND;
                clawAngle = RobotMap.CLAWJOINT_CORAL_SECOND_POLE_ANGLE;
                break;
            case THIRD_STAGE:
                armLength = RobotMap.ARM_LENGTH_THIRD;
                armAngle = RobotMap.ARM_JOINT_ANGLE_THIRD;
                clawAngle = RobotMap.CLAWJOINT_CORAL_THIRD_POLE_ANGLE;
                break;
            default:
                return null;
        }

        return createCommandGroupSimple(armLength, armAngle, clawAngle);
    }

    private Command createCommandGroupSimple(double armLength, double armAngle, double clawAngle) {
        return new ParallelCommandGroup(
                new MoveClawJointToPosition(clawJointSystem, clawAngle),
                new ArmTelescopicMoveToLength(armTelescopicSystem, armLength),
                Commands.runOnce(() -> armJointControlCommand.setTargetPosition(armAngle)),
                Commands.waitUntil(() -> armJointControlCommand.isAtTargetPosition())
        );
    }
}