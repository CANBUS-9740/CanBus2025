package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmJointSystem;
import frc.robot.subsystems.ArmTelescopicSystem;
import frc.robot.subsystems.ClawGripperSystem;
import frc.robot.subsystems.ClawJointSystem;
import frc.robot.subsystems.HangingSystem;
import frc.robot.subsystems.Swerve;

import java.util.Set;

public class Robot extends TimedRobot {

    private Swerve swerve;
    private ClawGripperSystem clawGripperSystem;
    private ClawJointSystem clawJointSystem;
    private ArmJointSystem armJointSystem;
    private ArmTelescopicSystem armTelescopicSystem;
    private HangingSystem hangingSystem;
    private ArmJointControlCommand armJointControlCommand;

    private XboxController xbox;

    @Override
    public void robotInit() {
        swerve = new Swerve();
        clawGripperSystem = new ClawGripperSystem();
        armJointSystem = new ArmJointSystem();
        clawJointSystem = new ClawJointSystem();
        armTelescopicSystem = new ArmTelescopicSystem();
        hangingSystem = new HangingSystem();

        armJointControlCommand = new ArmJointControlCommand(armJointSystem);

        armJointSystem.setDefaultCommand(
                new ArmJointControlCommand(armJointSystem)
        );
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


        xbox = new XboxController(0);


        Command collectFromSource = Commands.defer(()-> {
                    double sourceDistanceA = 0;
                    double sourceDistanceB = 0;
                    Pose2d robotPose = swerve.getPose();
                    if (isAllianceRed()) {
                        sourceDistanceA = swerve.getDistance(RobotMap.POSE_SOURCE_A_RED, robotPose);
                        sourceDistanceB = swerve.getDistance(RobotMap.POSE_SOURCE_B_RED, robotPose);
                    } else {
                        sourceDistanceA = swerve.getDistance(RobotMap.POSE_SOURCE_A_BLUE, robotPose);
                        sourceDistanceB = swerve.getDistance(RobotMap.POSE_SOURCE_B_BLUE, robotPose);
                    }
                    double targetsDistance;
                    if (sourceDistanceA > sourceDistanceB) {
                        targetsDistance = sourceDistanceA;
                    } else {
                        targetsDistance = sourceDistanceB;
                    }
                    double length = armTelescopicSystem.calculateLengthForTarget(targetsDistance, RobotMap.SOURCE_HEIGHT);
                    double angle = armJointSystem.calculateAngleForTarget(RobotMap.SOURCE_HEIGHT, targetsDistance);

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
                        distance = swerve.getDistance(RobotMap.POSE_PROCESSOR_RED, robotPose);
                    } else {
                        distance = swerve.getDistance(RobotMap.POSE_PROCESSOR_BLUE, robotPose);
                    }
                    double length = armTelescopicSystem.calculateLengthForTarget(distance, RobotMap.PROCESSOR_PLACE_HEIGHT);
                    double angle = armJointSystem.calculateAngleForTarget(RobotMap.PROCESSOR_PLACE_HEIGHT, distance);

                    if (isCommandIsValid(length, angle, distance)) {
                        return Commands.none();
                    }

                    return new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ArmTelescopicMoveToLength(armTelescopicSystem, length),
                                    Commands.runOnce(()->  armJointControlCommand.setTargetPosition(angle)),
                                    Commands.waitUntil(()-> armJointControlCommand.isAtTargetPosition()),
                                    new MoveClawJointToPosition(clawJointSystem, RobotMap.CLAWJOINT_SOURCE_ANGLE)),
                            new ClawGripperOuttake(clawGripperSystem)
                    );
                }, Set.of(armTelescopicSystem, clawJointSystem, clawGripperSystem)
        );
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();


    }

    public boolean isAllianceRed(){
        boolean isAllianceRed = false;
        if (DriverStation.getAlliance().isPresent()){
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                isAllianceRed = true;
            }
        }
        return isAllianceRed;
    }

    public boolean isCommandIsValid(double length, double angle, double distance) {
        return length > RobotMap.ARM_TELESCOPIC_MAXIMUM_LENGTH || length < RobotMap.ARM_TELESCOPIC_MINIMUM_LENGTH || angle < RobotMap.ARM_JOINT_MINIMUM_ANGLE || angle > RobotMap.ARM_JOINT_MAXIMUM_ANGLE || distance > RobotMap.ROBOT_MAXIMUM_DISTANCE;
    }

    public Command placeCoralOnReefCommand(CoralReef coralReef) {
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
        return Commands.defer(()->{
            Pose2d[][] coralPose = null;
            if (isAllianceRed()) {
                coralPose = RobotMap.POSE_CORAL_STANDS_RED;
            } else {
                coralPose = RobotMap.POSE_CORAL_STANDS_BLUE;
            }
            Pose2d robotPose = swerve.getPose();
            Pose2d stand = swerve.selectReefStand(coralPose, robotPose);
            double distance = swerve.getDistance(stand, robotPose);
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
                            new MoveClawJointToPosition(clawJointSystem, RobotMap.CLAWJOINT_SOURCE_ANGLE)),
                    new ClawGripperOuttake(clawGripperSystem)
            );
                }, Set.of(armTelescopicSystem, clawJointSystem, clawGripperSystem)
        );

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
    }

    @Override
    public void autonomousPeriodic() {


    }

    @Override
    public void autonomousExit() {

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
}
