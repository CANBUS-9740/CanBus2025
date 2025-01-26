package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmJointSystem;
import frc.robot.subsystems.ArmTelescopicSystem;
import frc.robot.subsystems.ClawGripperSystem;
import frc.robot.subsystems.ClawJointSystem;
import frc.robot.subsystems.HangingSystem;
import frc.robot.subsystems.Swerve;
import org.opencv.core.Mat;

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
        armJointSystem.setDefaultCommand(armJointControlCommand);

        xbox = new XboxController(0);


        Command collectSource = Commands.defer(()-> {
                    double sourceDistanceA = swerve.getDistance(RobotMap.POSE_SOURCE_A);
                    double sourceDistanceB = swerve.getDistance(RobotMap.POSE_SOURCE_B);
                    double targetsDistance;
                    if (sourceDistanceA > sourceDistanceB) {
                        targetsDistance = sourceDistanceA;
                    } else {
                        targetsDistance = sourceDistanceB;
                    }
                    double length = Math.sqrt(Math.pow(targetsDistance, 2) + Math.pow(RobotMap.SOURCE_HEIGHT, 2));
                    double angle = Math.atan(RobotMap.SOURCE_HEIGHT / targetsDistance);

                    if (length > RobotMap.ARM_TELESCOPIC_MAXIMUM_LENGTH || length < RobotMap.ARM_TELESCOPIC_MINIMUM_LENGTH || angle < RobotMap.ARM_JOINT_MINIMUM_ANGLE || angle > RobotMap.ARM_JOINT_MAXIMUM_ANGLE) {
                        return Commands.none();
                    }

                    return new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new ArmTelescopicMoveToLength(armTelescopicSystem, length),
                                    Commands.runOnce(()-> new ArmJointControlCommand(armJointSystem).setTargetPosition(angle)),
                                    Commands.waitUntil(()-> new ArmJointControlCommand(armJointSystem).isAtTargetPosition()),
                                    new ParallelDeadlineGroup(
                                            new InstantCommand(()-> clawJointSystem.isReachPosition(RobotMap.CLAWJOINT_SOURCE_ANGLE)),
                                            new MoveClawJointToPosition(clawJointSystem, RobotMap.CLAWJOINT_SOURCE_ANGLE))
                                    ),
                            new ClawGripperIntake(clawGripperSystem)

                            );






                }, Set.of()
        );
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();


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
