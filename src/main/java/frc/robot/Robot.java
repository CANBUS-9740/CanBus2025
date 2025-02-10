package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.Optional;
import java.util.Set;

import static frc.robot.RobotMap.isAllianceRed;

public class Robot extends TimedRobot {

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
                Commands.defer(()-> {
                    if (clawGripperSystem.hasItem()) {
                        return new HoldItemInClawGripper(clawGripperSystem);
                    }
                    return Commands.idle(clawGripperSystem);
                }, Set.of(clawGripperSystem))
        );

        Command collectFromSource = Commands.defer(()-> {
                    Pose2d robotPose = swerve.getPose();
                    Pose2d sourcePose = getClosestSource().getSecond();

                    double targetsDistance = swerve.getDistanceToMeters(robotPose, sourcePose);
                    double length = armTelescopicSystem.calculateLengthForTarget(targetsDistance, RobotMap.SOURCE_HEIGHT);
                    double angle = armJointSystem.calculateAngleForTarget(targetsDistance, RobotMap.SOURCE_HEIGHT);

                    if (isCommandIsValid(length, angle, targetsDistance)) {
                        return Commands.none();
                    }

                    return new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ArmTelescopicMoveToLength(armTelescopicSystem, length),
                                    Commands.runOnce(()->  armJointControlCommand.setTargetPosition(angle)),
                                    Commands.waitUntil(()->  armJointControlCommand.isAtTargetPosition()),
                                    new MoveClawJointToPosition(clawJointSystem, RobotMap.CLAWJOINT_SOURCE_ANGLE)),
                            new ClawGripperIntake(clawGripperSystem)
                            );
                }, Set.of(armTelescopicSystem, clawJointSystem, clawGripperSystem)
        );

        SequentialCommandGroup collectFromFloor = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        Commands.runOnce(()->  armJointControlCommand.setTargetPosition(RobotMap.ARM_JOINT_FLOOR_ANGLE)),
                        Commands.waitUntil(()-> armJointControlCommand.isAtTargetPosition()),
                        new MoveClawJointToPosition(clawJointSystem, RobotMap.CLAWJOINT_FLOOR_ANGLE)
                ),
                new ClawGripperIntake(clawGripperSystem)
        );

        Command placeOnReefPodium = placeCoralOnReefCommand(CoralReef.PODIUM);
        Command placeOnReefFirstStage = placeCoralOnReefCommand(CoralReef.FIRST_STAGE);
        Command placeOnReefSecondStage = placeCoralOnReefCommand(CoralReef.SECOND_STAGE);
        Command placeOnReefThirdStage = placeCoralOnReefCommand(CoralReef.THIRD_STAGE);

        Command placeInProcessor = Commands.defer(()-> {
                    double distance = 0;

                    Pose2d robotPose = swerve.getPose();

                    if (isAllianceRed()) {
                        distance = swerve.getDistanceToMeters(robotPose, RobotMap.POSE_PROCESSOR_RED);
                    } else {
                        distance = swerve.getDistanceToMeters(robotPose, RobotMap.POSE_PROCESSOR_BLUE);
                    }

                    double length = armTelescopicSystem.calculateLengthForTarget(distance, RobotMap.PROCESSOR_PLACE_HEIGHT);
                    double angle = armJointSystem.calculateAngleForTarget(distance, RobotMap.PROCESSOR_PLACE_HEIGHT);

                    if (isCommandIsValid(length, angle, distance)) {
                        return Commands.none();
                    }

                    return new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ArmTelescopicMoveToLength(armTelescopicSystem, length),
                                    Commands.runOnce(()->  armJointControlCommand.setTargetPosition(angle)),
                                    Commands.waitUntil(()-> armJointControlCommand.isAtTargetPosition()),
                                    new MoveClawJointToPosition(clawJointSystem, RobotMap.CLAWJOINT_PROCESSOR_ANGLE)),
                            new ClawGripperOuttake(clawGripperSystem)
                    );
                }, Set.of(armTelescopicSystem, clawJointSystem, clawGripperSystem)
        );

        FollowPathCommand.warmupCommand().schedule();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        Command alignWithSource = Commands.defer(()->{
            Pose2d sourcePose = getClosestSource().getSecond();
            sourcePose = new Pose2d(sourcePose.getX(),sourcePose.getY(), Rotation2d.fromRadians(Math.toRadians(sourcePose.getRotation().getDegrees()+180)));

            return AutoBuilder.followPath(swerve.getFollowPathToTarget(sourcePose));
        }, Set.of(swerve));

        Command alignWithProcessor = Commands.defer(()->{
            Pose2d processorPose = RobotMap.isAllianceRed() ? RobotMap.POSE_INFRONT_PROCESSOR_RED : RobotMap.POSE_INFRONT_PROCESSOR_BLUE;
            return AutoBuilder.followPath(swerve.getFollowPathToTarget(processorPose));
        }, Set.of(swerve));

        Command alignWithCoralStand = Commands.defer(()->{
            Optional<SelectedStand> optionalStand = getClosestStand();
            if (optionalStand.isEmpty()) {
                return Commands.none();
            }
            Pose2d stand = optionalStand.get().pose;
            return AutoBuilder.followPath(swerve.getFollowPathToTarget(stand));
        }, Set.of(swerve));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        Optional<SelectedStand> standOptional = getBestStand();
        if (standOptional.isPresent()) {
            SelectedStand stand = standOptional.get();
            swerve.getField().getObject("BestStand").setPose(stand.pose);
            SmartDashboard.putNumber("BestStand", stand.index);
            SmartDashboard.putNumber("BestStandRow", stand.row);
            SmartDashboard.putBoolean("HasBestStand", true);
        } else {
            swerve.getField().getObject("BestStand").setPoses();
            SmartDashboard.putNumber("BestStand", -1);
            SmartDashboard.putNumber("BestStandRow", -1);
            SmartDashboard.putBoolean("HasBestStand", false);
        }

        standOptional = getClosestStand();
        if (standOptional.isPresent()) {
            SelectedStand stand = standOptional.get();
            swerve.getField().getObject("ClosestStand").setPose(stand.pose);
            SmartDashboard.putNumber("ClosestStand", stand.index);
            SmartDashboard.putNumber("ClosestStandRow", stand.row);
        } else {
            swerve.getField().getObject("ClosestStand").setPoses();
            SmartDashboard.putNumber("ClosestStand", -1);
            SmartDashboard.putNumber("ClosestStandRow", -1);
        }

        Pair<Integer, Pose2d> closestSource = getClosestSource();
        SmartDashboard.putNumber("ClosestSource", closestSource.getFirst());
        swerve.getField().getObject("ClosestSource").setPose(closestSource.getSecond());
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
                ()-> -MathUtil.applyDeadband(Math.pow(xbox.getRightY(),3), 0.05),
                ()-> MathUtil.applyDeadband(Math.pow( xbox.getRightX(),3), 0.05),
                ()-> MathUtil.applyDeadband(xbox.getLeftX() , 0.15)
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
        if(auto != null){
            auto.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        if(auto != null){
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

    private boolean isCommandIsValid(double length, double angle, double distance) {
        return length > RobotMap.ARM_TELESCOPIC_MAXIMUM_LENGTH ||
                length < RobotMap.ARM_TELESCOPIC_MINIMUM_LENGTH ||
                angle < RobotMap.ARM_JOINT_MINIMUM_ANGLE ||
                angle > RobotMap.ARM_JOINT_MAXIMUM_ANGLE ||
                distance > RobotMap.ROBOT_MAXIMUM_DISTANCE;
    }

    private Optional<SelectedStand> getBestStand() {
        Pose2d pose = swerve.getPose();
        Pose2d[][] stands = isAllianceRed() ? RobotMap.POSE_CORAL_STANDS_RED : RobotMap.POSE_CORAL_STANDS_BLUE;
        return swerve.findBestStand(pose, stands, true);
    }

    private Optional<SelectedStand> getClosestStand() {
        Pose2d pose = swerve.getPose();
        Pose2d[][] stands = isAllianceRed() ? RobotMap.POSE_CORAL_STANDS_RED : RobotMap.POSE_CORAL_STANDS_BLUE;
        return swerve.findBestStand(pose, stands, false);
    }

    public Pair<Integer, Pose2d> getClosestSource() {
        Pose2d robotPose = swerve.getPose();

        Pose2d sourceA = isAllianceRed() ? RobotMap.POSE_SOURCE_A_RED : RobotMap.POSE_SOURCE_A_BLUE;
        Pose2d sourceB = isAllianceRed() ? RobotMap.POSE_SOURCE_B_RED : RobotMap.POSE_SOURCE_B_BLUE;

        double sourceDistanceA = swerve.getDistanceToMeters(robotPose, sourceA);
        double sourceDistanceB = swerve.getDistanceToMeters(robotPose, sourceB);
        if (sourceDistanceA < sourceDistanceB) {
            return Pair.of(0, sourceA);
        } else {
            return Pair.of(1, sourceB);
        }
    }

    private Command placeCoralOnReefCommand(CoralReef coralReef) {
        double reefPoleHeight;
        double crawJointAngle;
        switch (coralReef){
            case PODIUM:
                reefPoleHeight = RobotMap.CORAL_PODIUM_POLE_HEIGHT;
                crawJointAngle = RobotMap.CLAWJOINT_CORAL_PODIUM_POLE_ANGLE;
                break;
            case FIRST_STAGE:
                reefPoleHeight = RobotMap.CORAL_LOWER_POLE_HEIGHT;
                crawJointAngle = RobotMap.CLAWJOINT_CORAL_LOWER_POLE_ANGLE;
                break;
            case SECOND_STAGE:
                reefPoleHeight = RobotMap.CORAL_MEDIUM_POLE_HEIGHT;
                crawJointAngle = RobotMap.CLAWJOINT_CORAL_MEDIUM_POLE_ANGLE;
                break;
            case THIRD_STAGE:
                reefPoleHeight = RobotMap.CORAL_HIGH_POLE_HEIGHT;
                crawJointAngle = RobotMap.CLAWJOINT_CORAL_HIGH_POLE_ANGLE;
                break;
            default:
                reefPoleHeight = 0;
                crawJointAngle = 0;
                break;
        }

        return Commands.defer(()-> {
            Optional<SelectedStand> standOptional = getBestStand();
            if (standOptional.isEmpty()) {
                return Commands.none();
            }

            Pose2d stand = standOptional.get().pose;
            Pose2d robotPose = swerve.getPose();

            double distance = swerve.getDistanceToMeters(robotPose, stand);
            double length = armTelescopicSystem.calculateLengthForTarget(distance, reefPoleHeight);
            double angle = armJointSystem.calculateAngleForTarget(distance, reefPoleHeight);

            if (isCommandIsValid(length, angle, distance)) {
                return Commands.none();
            }

            return new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new ArmTelescopicMoveToLength(armTelescopicSystem, length),
                            Commands.runOnce(()-> armJointControlCommand.setTargetPosition(angle)),
                            Commands.waitUntil(()-> armJointControlCommand.isAtTargetPosition()),
                            new MoveClawJointToPosition(clawJointSystem, crawJointAngle)),
                    new ClawGripperOuttake(clawGripperSystem)
            );
        }, Set.of(armTelescopicSystem, clawJointSystem, clawGripperSystem));
    }
}
