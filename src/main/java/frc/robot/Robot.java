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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.Swerve;

import java.util.Locale;
import java.util.Optional;
import java.util.Set;

public class Robot extends TimedRobot {

    private GameField gameField;
    private Swerve swerve;
    private Command auto;

    private IntakeSystem intakeSystem;

    private XboxController controllerXbox;
    private XboxController driverXbox;
    private SendableChooser<Command> autoChooser;

    @Override
    public void robotInit() {
        gameField = new GameField();
        swerve = new Swerve();
        intakeSystem = new IntakeSystem();
        driverXbox = new XboxController(0);
        controllerXbox = new XboxController(1);


        swerve.setDefaultCommand(swerve.drive(
                () -> -MathUtil.applyDeadband(Math.pow(driverXbox.getRightY(), 3), 0.05),
                () -> MathUtil.applyDeadband(Math.pow(-driverXbox.getRightX(), 3), 0.05),
                () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.15)
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

        new JoystickButton(controllerXbox, XboxController.Button.kB.value).onTrue(
                new OuttakeCommand(intakeSystem)
        );

        new JoystickButton(controllerXbox, XboxController.Button.kX.value).onTrue(
                new IntakeCommand(intakeSystem)
        );


//        new JoystickButton(xbox, XboxController.Button.kLeftBumper.value).onTrue(
//                new ArmTelescopicReset(armTelescopicSystem)
//        );

        /* new JoystickButton(xbox, XboxController.Button.kRightBumper.value).onTrue(
                new ArmTelescopicMoveToLength(armTelescopicSystem, 0.7)
        );
        when we csn extend our arl return it
        */

        FollowPathCommand.warmupCommand().schedule();
        autoChooser = new SendableChooser<>();
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
}