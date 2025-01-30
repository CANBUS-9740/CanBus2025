package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ArmJointControlCommand;
import frc.robot.dashboard.Dashboard;
import frc.robot.dashboard.Tab;
import frc.robot.dashboard.layout.Column;
import frc.robot.dashboard.layout.WeightedRow;
import frc.robot.dashboard.widget.BooleanWidget;
import frc.robot.dashboard.widget.FieldWidget;
import frc.robot.subsystems.ArmJointSystem;
import frc.robot.subsystems.ArmTelescopicSystem;
import frc.robot.subsystems.ClawGripperSystem;
import frc.robot.subsystems.ClawJointSystem;
import frc.robot.subsystems.HangingSystem;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {

    private Swerve swerve;
    private ClawGripperSystem clawGripperSystem;
    private ClawJointSystem clawJointSystem;
    private ArmJointSystem armJointSystem;
    private ArmTelescopicSystem armTelescopicSystem;
    private HangingSystem hangingSystem;

    private Dashboard dashboard;

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

        dashboard = new Dashboard(
                new Dashboard.Configuration(13, 6),
                new Tab(
                        "Generated Tab",
                        "generated-tab",
                        new WeightedRow(
                                new float[]{0.5f, 0.25f, 0.25f},
                                new FieldWidget("Estimated Position", swerve.getField()),
                                new Column(
                                        new BooleanWidget("Boolean Widget 2", () -> false),
                                        new BooleanWidget("Boolean Widget 3", () -> true),
                                        new BooleanWidget("Boolean Widget 4", () -> false)
                                ),
                                new Column(
                                        new BooleanWidget("Boolean Widget 5", () -> true),
                                        new BooleanWidget("Boolean Widget 6", () -> false)
                                )
                        )
                )
        );
        dashboard.setup();
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
