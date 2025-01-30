package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArmJointControlCommand;
import frc.robot.subsystems.ArmJointSystem;
import frc.robot.subsystems.ArmTelescopicSystem;
import frc.robot.subsystems.ClawGripperSystem;
import frc.robot.subsystems.ClawJointSystem;
import frc.robot.subsystems.HangingSystem;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {

    private Swerve swerve;
    /*
    private ClawGripperSystem clawGripperSystem;
    private ClawJointSystem clawJointSystem;
    private ArmJointSystem armJointSystem;
    private ArmTelescopicSystem armTelescopicSystem;
    private HangingSystem hangingSystem;

    private ArmJointControlCommand armJointControlCommand;
*/
    private XboxController xbox;
    private SendableChooser<Command> autoChooser;

    @Override
    public void robotInit() {
        swerve = new Swerve();
        SmartDashboard.putData("Swerve", swerve);
        /*clawGripperSystem = new ClawGripperSystem();
        armJointSystem = new ArmJointSystem();
        clawJointSystem = new ClawJointSystem();
        armTelescopicSystem = new ArmTelescopicSystem();
        hangingSystem = new HangingSystem();

        armJointControlCommand = new ArmJointControlCommand(armJointSystem);
        armJointSystem.setDefaultCommand(armJointControlCommand);
*/
        xbox = new XboxController(0);

        FollowPathCommand.warmupCommand().schedule();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
        swerve.drive(
                ()-> MathUtil.applyDeadband(-xbox.getRightY(), 0.05),
                ()-> MathUtil.applyDeadband(xbox.getRightX(), 0.05),
                ()-> MathUtil.applyDeadband(xbox.getLeftX(), 0.05)
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
        swerve.resetPose(Pose2d.kZero);
        autoChooser.getSelected().schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {
        swerve.drive(()-> 0, ()-> 0, ()-> 0.4).schedule();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }
}
