package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.commands.*;
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
    private ClawJointControlCommand clawJointControlCommand;

    private XboxController controllerXbox;
    private XboxController driverXbox;
    private SendableChooser<Command> autoChooser;

    @Override
    public void robotInit() {
        gameField = new GameField();
        swerve = new Swerve();
        clawGripperSystem = new ClawGripperSystem();
        //armJointSystem = new ArmJointSystem();
        //clawJointSystem = new ClawJointSystem();
        //armTelescopicSystem = new ArmTelescopicSystem();

        driverXbox = new XboxController(0);
        controllerXbox = new XboxController(1);

        //armJointControlCommand = new ArmJointControlCommand(armJointSystem, armTelescopicSystem);
        //armJointSystem.setDefaultCommand(armJointControlCommand);

        //clawJointControlCommand = new ClawJointControlCommand(armJointSystem, clawJointSystem);
        //clawJointSystem.setDefaultCommand(clawJointControlCommand);

        swerve.setDefaultCommand(swerve.fieldDrive(
                () -> MathUtil.applyDeadband(Math.pow(driverXbox.getRightY(), 3), 0.05),
                () -> MathUtil.applyDeadband(Math.pow(driverXbox.getRightX(), 3), 0.05),
                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), 0.15)
        ));

        /*armTelescopicSystem.setDefaultCommand(
                Commands.defer(()-> {
                    if (armTelescopicSystem.getResetLimitSwitch()) {
                        return Commands.idle(armTelescopicSystem);
                    }

                    return new ArmTelescopicReset(armTelescopicSystem);
                }, Set.of(armTelescopicSystem))
        );
        clawGripperSystem.setDefaultCommand(
                Commands.defer(() -> {
                    if (clawGripperSystem.hasItem()) {
                        return new HoldItemInClawGripper(clawGripperSystem);
                    }
                    return Commands.idle(clawGripperSystem);
                }, Set.of(clawGripperSystem))
        );*/

//        SequentialCommandGroup collectFromFloor = new SequentialCommandGroup(
//                createCommandGroupSimple(RobotMap.ARM_LENGTH_FLOOR, RobotMap.ARM_JOINT_FLOOR_ANGLE, RobotMap.CLAWJOINT_FLOOR_ANGLE),
//                new ClawGripperIntake(clawGripperSystem)
//        );
//
//        ParallelCommandGroup underCage = new ParallelCommandGroup(
//                new ArmTelescopicReset(armTelescopicSystem),
//                Commands.runOnce(()->  armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_UNDER_CAGE_ANGLE)),
//                Commands.runOnce(()-> clawJointControlCommand.setTargetPosition(RobotMap.CLAWJOINT_UNDER_CAGE_ANGLE))
//        );
//
//        ParallelCommandGroup hang = new ParallelCommandGroup(
//                new ArmTelescopicReset(armTelescopicSystem),
//                Commands.runOnce(()->  armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_FLOOR_ANGLE))
//        );
//
//        SequentialCommandGroup reefLowerAlgae = new SequentialCommandGroup(
//                createCommandGroupSimple(RobotMap.ARM_TELESCOPIC_LOWER_REEF_ALGAE_LENGTH, RobotMap.ARM_JOINT_LOWER_REEF_ALGAE_ANGLE, RobotMap.CLAWJOINT_LOWER_REEF_ALGAE_ANGLE),
//                new ClawGripperIntake(clawGripperSystem)
//        );
//
//        SequentialCommandGroup reefHighAlgae = new SequentialCommandGroup(
//                createCommandGroupSimple(RobotMap.ARM_TELESCOPIC_HIGH_REEF_ALGAE_LENGTH, RobotMap.ARM_JOINT_HIGH_REEF_ALGAE_ANGLE, RobotMap.CLAWJOINT_HIGH_REEF_ALGAE_ANGLE),
//                new ClawGripperIntake(clawGripperSystem)
//        );
//
//        Command collectFromSourceCommandSimple =
//                createCommandGroupSimple(0.5, RobotMap.ARM_JOINT_ANGLE_SOURCE, RobotMap.CLAWJOINT_SOURCE_ANGLE);
//        Command placeInProcessorCommandSimple =
//                createCommandGroupSimple(RobotMap.CALCULATION_PLACE_IN_PROCESSOR, RobotMap.ANGLE_PROCESSOR, RobotMap.CLAWJOINT_PROCESSOR_ANGLE);

        //new POVButton(xbox, 180).onTrue(placeCoralOnReefCommandSimple(CoralReef.PODIUM));
        //new POVButton(xbox, 270).onTrue(placeCoralOnReefCommandSimple(CoralReef.FIRST_STAGE));
        //new POVButton(xbox, 90).onTrue(placeCoralOnReefCommandSimple(CoralReef.SECOND_STAGE));
        //new POVButton(xbox, 0).onTrue(placeCoralOnReefCommandSimple(CoralReef.THIRD_STAGE));

        //new JoystickButton(xbox, XboxController.Button.kRightBumper.value).onTrue(placeInProcessorCommandSimple);
        //new JoystickButton(xbox, XboxController.Button.kLeftBumper.value).onTrue(collectFromSourceCommandSimple);
        //new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new ClawGripperOuttake(clawGripperSystem));
        //new JoystickButton(xbox, XboxController.Button.kX.value).onTrue(new ClawGripperIntake(clawGripperSystem));
        //new JoystickButton(xbox, XboxController.Button.kA.value).onTrue(reefHighAlgae);
        //new JoystickButton(xbox, XboxController.Button.kY.value).onTrue(placeInProcessor);
//        new JoystickButton(xbox, XboxController.Button.kY.value).onTrue(
//                Commands.defer(()-> {
//                    double length = armTelescopicSystem.getLengthMeters();
//                    return new ArmTelescopicMoveToLength(armTelescopicSystem, length + 0.1);
//                }, Set.of(armTelescopicSystem))
//        );
//        new JoystickButton(xbox, XboxController.Button.kA.value).onTrue(
//                Commands.defer(()-> {
//                    double length = armTelescopicSystem.getLengthMeters();
//                    return new ArmTelescopicMoveToLength(armTelescopicSystem, length - 0.1);
//                }, Set.of(armTelescopicSystem))
//        );


//        new JoystickButton(xbox, XboxController.Button.kY.value).whileTrue(
//                new ClawGripperOuttakeSlow(clawGripperSystem)
//        );

//        new JoystickButton(controllerXbox, XboxController.Button.kA.value).whileTrue(
//                placeCoralOnReefCommandSimple(CoralReef.PODIUM)
//        );

//        new JoystickButton(controllerXbox, XboxController.Button.kY.value).whileTrue(
//                new ParallelCommandGroup(
//                        Commands.runOnce(()-> armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE)),
//                        Commands.waitUntil(()-> armJointControlCommand.isAtTargetPosition()),
//                        Commands.runOnce(()-> clawJointControlCommand.setTargetPosition(RobotMap.CLAWJOINT_DEFAULT_ANGLE)),
//                        Commands.waitUntil(()-> clawJointControlCommand.isAtTargetPosition()),
//                        Commands.runOnce(()-> System.out.println("going to default"))
//                )
//        );

//        new ParallelCommandGroup(
//                Commands.runOnce(()-> armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE)),
//                Commands.waitUntil(()-> armJointControlCommand.isAtTargetPosition()),
//                Commands.runOnce(()-> clawJointControlCommand.setTargetPosition(RobotMap.CLAWJOINT_DEFAULT_ANGLE)),
//                Commands.waitUntil(()-> clawJointControlCommand.isAtTargetPosition()),
//                Commands.runOnce(()-> System.out.println("going to default"))
//        );

        /*new JoystickButton(controllerXbox, XboxController.Button.kStart.value).whileTrue(
                new ParallelCommandGroup(
                        Commands.runOnce(()-> armJointControlCommand.setTargetPosition(40)),
                        Commands.waitUntil(()-> armJointControlCommand.isAtTargetPosition()),
                        Commands.runOnce(()-> clawJointControlCommand.setTargetPosition(31)),
                        Commands.waitUntil(()-> clawJointControlCommand.isAtTargetPosition()),
                        Commands.runOnce(()-> System.out.println("going to floor"))
                )
        );*/

        new JoystickButton(controllerXbox, XboxController.Button.kB.value).whileTrue(
                new ClawGripperOuttake(clawGripperSystem)
        );

        new JoystickButton(controllerXbox, XboxController.Button.kX.value).whileTrue(
                new ClawGripperIntake(clawGripperSystem)
        );


//        new JoystickButton(controllerXbox, XboxController.Button.kLeftBumper.value).whileTrue(
//                placeCoralOnReefCommandSimple(CoralReef.FIRST_STAGE)
//        );

//        new JoystickButton(controllerXbox, XboxController.Button.kRightBumper.value).whileTrue(
//                collectFromSourceCommandSimple
//        );

//        new JoystickButton(controllerXbox, XboxController.Button.kLeftBumper.value).onTrue(
//                new ArmTelescopicReset(armTelescopicSystem)
//                );

        /* new JoystickButton(xbox, XboxController.Button.kRightBumper.value).onTrue(
                new ArmTelescopicMoveToLength(armTelescopicSystem, 0.7)
        );
        when we csn extend our arl return it
        */


//        new POVButton(controllerXbox, 0).onTrue(
//                Commands.runOnce(()-> {
//                    double oldAngle = armJointControlCommand.getTargetPosition();
//                    if (oldAngle < 0) {
//                        oldAngle = armJointSystem.getRawPositionDegrees();
//                    }
//                    armJointControlCommand.setTargetPosition(oldAngle + 5);
//                })
//        );
//
//        new POVButton(controllerXbox, 180).onTrue(
//                Commands.runOnce(()-> {
//                    double oldAngle = armJointControlCommand.getTargetPosition();
//                    if (oldAngle < 0) {
//                        oldAngle = armJointSystem.getRawPositionDegrees();
//                    }
//                    armJointControlCommand.setTargetPosition(oldAngle - 5);
//                })
//        );
//
//        new POVButton(controllerXbox, 90).onTrue(
//                Commands.runOnce(()-> {
//                    double oldAngle = clawJointControlCommand.getTargetPosition();
//                    if (oldAngle < 0) {
//                        oldAngle = clawJointSystem.getRawPositionDegrees();
//                    }
//                    clawJointControlCommand.setTargetPosition(oldAngle + 5);
//                })
//        );
//        new POVButton(controllerXbox, 270).onTrue(
//                Commands.runOnce(()-> {
//                    double oldAngle = clawJointControlCommand.getTargetPosition();
//                    if (oldAngle < 0) {
//                        oldAngle = clawJointSystem.getRawPositionDegrees();
//                    }
//                    clawJointControlCommand.setTargetPosition(oldAngle - 5);
//                })
//        );

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
        }, Set.of(armTelescopicSystem, clawGripperSystem));
    }

    private Command placeInProcessor() {
        return Commands.defer(() -> {
            Pose2d robotPose = swerve.getPose();
            Pose2d processorPose = gameField.getPoseToProcessor();
            double distance = swerve.getDistanceToMeters(robotPose, processorPose);

            double angle = armJointSystem.calculateAngleForTarget(distance, RobotMap.PROCESSOR_PLACE_HEIGHT);

            if (isCommandNotValid(angle, distance)) {
                return Commands.none();
            }

            return new SequentialCommandGroup(
                    createCommandGroupSimple(angle),
                    new ClawGripperOuttake(clawGripperSystem),
                    Commands.runOnce(()-> armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_DEFAULT_ANGLE))
            );
        }, Set.of(armTelescopicSystem, clawGripperSystem));
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
        }, Set.of(armTelescopicSystem, clawGripperSystem));
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