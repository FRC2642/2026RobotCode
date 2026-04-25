// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.function.ToDoubleBiFunction;
import java.util.function.ToDoubleFunction;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.IntakeSpin;
import frc.robot.subsystems.Intermediate;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climby;
import frc.robot.subsystems.intakeTilt;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.intakeTilt.RotationPositions;
@SuppressWarnings("unused")

public class RobotContainer {

    private PathPlannerAuto auto;
    private final CommandJoystick buttonBoard = new CommandJoystick(Constants.BUTTON_BOARD_PORT);
    private final CommandXboxController controller = new CommandXboxController(Constants.MAIN_CONTROLLER_PORT);
    private final CommandXboxController auxController = new CommandXboxController(Constants.AUX_CONTROLLER_PORT);

    private double MaxSpeed = 1 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate *0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Intermediate intermediate = new Intermediate();
    public final Vision vision = new Vision(drivetrain);
    public final intakeTilt intakeTilt = new intakeTilt();
    public final IntakeSpin intakeSpin = new IntakeSpin();
    public final shooter shooterSub = new shooter();

    public final Climby climby = new Climby();
    public final Dashboard dash = new Dashboard(vision, controller, intakeTilt);

    public final SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        drivetrain.ConfigureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.setDefaultOption("Disruptor Auto 1", new PathPlannerAuto("Disruptor Auto 1"));
        configureBindings();
    }
    private void configureBindings() {
    //DEFAULT SWERVE
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(controller.getLeftY() * Climby.constrain(controller.getLeftTriggerAxis()+0.5, 0 ,1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(controller.getLeftX() * Climby.constrain(controller.getLeftTriggerAxis()+0.5, 0 ,1) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * Climby.constrain(controller.getLeftTriggerAxis()+0.7, 0 ,1) * MaxAngularRate))); // Drive counterclockwise with negative X (left)
    //SPIN
        controller.a().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(controller.getLeftY() * Climby.constrain(controller.getLeftTriggerAxis()+0.5, 0 ,1) * MaxSpeed)
                    .withVelocityY(controller.getLeftX() * Climby.constrain(controller.getLeftTriggerAxis()+0.5, 0 ,1) * MaxSpeed) 
                    .withRotationalRate(MaxAngularRate)));
    //RESET GYRO
        controller.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        controller.y().onTrue(vision.updatePose());
    //AUTO AIM
        controller.b().whileTrue(
            AutoBuilder.pathfindToPose(
                new Pose2d(1.5, 4, Rotation2d.fromDegrees(0)), //Target Pose
                new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                0)
            );
        // controller.b().whileTrue(
        //     drivetrain.applyRequest(()->
        //     robotDrive.withVelocityX(-vision.getDriveOutput())
        //         .withVelocityY(vision.getRotateOutput())
        //         .withRotationalRate(-vision.getRotateOutput())));
    //SHOOT
        auxController.leftTrigger().whileTrue(shooterSub
            .runShooterWheels(Constants.SHOOTER_ROLLER_1_SPEED, 
                                Constants.SHOOTER_ROLLER_2_SPEED, 
                                Constants.SHOOTER_FLYWHEEL_SPEED)
                .alongWith(intermediate.Spin(Constants.INTERMEDIATE_SPEED)));
    //START UP FLYWHEEL
        auxController.y().whileTrue(shooterSub
            .runShooterWheels(0, 0, 
                            Constants.START_FLYWHEEL_SPEED));
    //PASS
        auxController.leftTrigger().whileTrue(shooterSub
            .runShooterWheels(Constants.SHOOTER_ROLLER_1_SPEED, 
                                Constants.SHOOTER_ROLLER_2_SPEED, 
                                Constants.PASSING_FLYWHEEL_SPEED)
                .alongWith(intermediate.Spin(Constants.INTERMEDIATE_SPEED)));

    {//Shooter testing (12,11,10,8,7,1)
        // buttonBoard.button(12).whileTrue(shooterSub.TestShooterMotors(1, 1));
        // buttonBoard.button(11).whileTrue(shooterSub.TestShooterMotors(2, 1));
        // buttonBoard.button(10).whileTrue(shooterSub.TestShooterMotors(3, 1));
        // buttonBoard.button(8).whileTrue(shooterSub.TestShooterMotors(4, 1));
        // buttonBoard.button(7).whileTrue(shooterSub.TestShooterMotors(5, 1));
        //controller.y().whileTrue(shooterSub.TestShooterMotors(6, 1));
}

    //TESTING
        controller.x().whileTrue(intermediate.Spin(0.3));
        controller.povRight().whileTrue(shooterSub.runShooterWheels(0.6,0,0));
        controller.povLeft().whileTrue(shooterSub.runShooterWheels(0,0.6,0));

    //INTAKE TOGGLE
        auxController.a().onTrue(intakeTilt.toggleRotate());
    //INTAKE PULSE
        //auxController.x().whileTrue(intakeTilt.Pulse());
    //INTAKE SPIN
        auxController.b().whileTrue(intakeSpin.spin(Constants.INTAKE_SPIN_SPEED));
    //MANUAL INTAKE TILT
        //UP
        auxController.rightBumper().whileTrue(intakeTilt.manualIntake(Constants.MANUAL_TILT_SPEED));
        //DOWN
        auxController.leftBumper().whileTrue(intakeTilt.manualIntake(-Constants.MANUAL_TILT_SPEED));
    //REVERSE REVERSE
        auxController.povUp().whileTrue((shooterSub.runShooterWheels(Constants.REVERSE_SHOOTER_SPEED, 
                                                                    Constants.REVERSE_SHOOTER_SPEED, 
                                                                    Constants.REVERSE_SHOOTER_SPEED)
                            .alongWith(intermediate.Spin(-Constants.INTERMEDIATE_SPEED))));

    //what does any of this do? Who knows. I'm not gonna touch it tho
        {final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        controller.back().and(controller.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        controller.back().and(controller.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        drivetrain.registerTelemetry(logger::telemeterize);
        }
    }
    public Command getAutonomousCommand() {
        //return new PathPlannerAuto("Taxi Auto");
        return autoChooser.getSelected();
        //return Commands.print("No auto enabled");
    }
    
}
