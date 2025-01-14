package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Swerve;
import swervelib.SwerveModule;

public class Robot extends TimedRobot {

    private Swerve swerve;
    private XboxController xbox;

    @Override
    public void robotInit() {
        swerve = new Swerve();
        xbox = new XboxController(0);
        swerve.centerModules().schedule();
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
                ()-> MathUtil.applyDeadband(-xbox.getRightX(), 0.05),
                ()-> MathUtil.applyDeadband(-xbox.getLeftX(), 0.05)
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
        swerve.drive(
                ()-> MathUtil.applyDeadband(0.05, 0.0),
                ()-> MathUtil.applyDeadband(0, 0.0),
                ()-> MathUtil.applyDeadband(0, 0.0)
        ).schedule();
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void testInit() {
//        swerve.drive(()-> 0.0, ()-> 0.0, ()-> -0.6).schedule();
        swerve.centerModules().schedule();
        //swerve.drive(()-> 0.2, ()-> 0.0, ()-> 0.0).schedule();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {

    }
}
