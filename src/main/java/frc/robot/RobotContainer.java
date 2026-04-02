// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.subsystems.shooterTilt;
import frc.robot.subsystems.shooterTilt.tiltStates;
import frc.robot.subsystems.intakeTilt.RotationPositions;
@SuppressWarnings("unused")

public class RobotContainer {

    private PathPlannerAuto auto;

    private final CommandJoystick buttonBoard = new CommandJoystick(2);
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController auxController = new CommandXboxController(1);

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
    public final Vision vision = new Vision();
    public final intakeTilt intakeTilt = new intakeTilt();
    public final IntakeSpin intakeSpin = new IntakeSpin();
    public final shooter shooterSub = new shooter();
    public final shooterTilt shooterTiltSub = new shooterTilt();
    public final Climby climby = new Climby();
    public final Dashboard dash = new Dashboard(vision, controller, intakeTilt);

   private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        

        // make the autos so they show up in the auto selector
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Taxi", new PathPlannerAuto("Taxi Auto"));
        autoChooser.addOption("Shoot", new PathPlannerAuto("Shoot Auto"));
        SmartDashboard.putData("Auto Chooser", autoChooser);
        //create named commands
        //these are commands to perform certain actions during auto
        NamedCommands.registerCommand("shoot", shooterSub.staticShoot(.8, .7));
        NamedCommands.registerCommand("intermediate", intermediate.Spin(.3));
        configureBindings();
    }
    private void configureBindings() {
    //DEFAULT SWERVE
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(controller.getLeftY() * Climby.constrain(controller.getLeftTriggerAxis()+0.5, 0 ,1) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(controller.getLeftX() * Climby.constrain(controller.getLeftTriggerAxis()+0.5, 0 ,1) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * Climby.constrain(controller.getLeftTriggerAxis()+0.5, 0 ,1) * MaxAngularRate))); // Drive counterclockwise with negative X (left)

        auxController.povUp().whileTrue(drivetrain.applyRequest(() ->
                drive.withVelocityX(controller.getLeftY() * Climby.constrain(controller.getLeftTriggerAxis()+0.5, 0 ,1) * MaxSpeed)
                    .withVelocityY(controller.getLeftX() * Climby.constrain(controller.getLeftTriggerAxis()+0.5, 0 ,1) * MaxSpeed) 
                    .withRotationalRate(MaxAngularRate)));
    //RESET GYRO
        controller.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        //controller.rightTrigger().onTrue(intakeTilt.resetEncoder());
    //AUTO AIM
        controller.y().whileTrue(
            drivetrain.applyRequest(()->
            robotDrive.withVelocityX(vision.getDriveOutput())
                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                .withRotationalRate(vision.getRotateOutput())));
    //SHOOT
        controller.rightBumper().whileTrue(shooterSub.staticShoot(0.8,0.7)
                                .alongWith(intermediate.Spin(0.75)));
        controller.b().toggleOnTrue(shooterSub.runShooterWheels(1));

    {//(NOT USED FOR WAKE COMP)
        //controller.a().whileTrue(shooterTiltSub.manualTilt(0.1));
    //(NOT USED FOR WAKE COMP)
        //controller.rightBumper().onTrue(intakeTilt.rotateToShoot());
    //(NOT USED FOR WAKE COMP)
        //controller.b().onTrue(shooterTiltSub.tilt(tiltStates.top));
    }
    //REVERSE REVERSE
        auxController.x().whileTrue((shooterSub.staticShoot(-0.5, -0.5)
                                .alongWith(intermediate.Spin(-0.75))));
    //INTERMEDIATE
        auxController.leftTrigger().whileTrue(intermediate.Spin(-0.75));
        auxController.rightTrigger().whileTrue(intermediate.Spin(0.75));
    //INTAKE TOGGLE
        auxController.a().onTrue(intakeTilt.toggleRotate());
        auxController.b().whileTrue(intakeSpin.spin(0.40)
                                .alongWith(intermediate.Spin(0.75)));
        auxController.y().whileTrue(intakeSpin.spin(-0.40));
    //MANUAL INTAKE TILT
        auxController.rightBumper().whileTrue(
        intakeTilt.manualIntake(0.5));
        auxController.leftBumper().whileTrue(intakeTilt.manualIntake(-0.5));
    {//CLIMB ALLIGNMENT (NATE) 
    //(NOT USED FOR WAKE COMP)
        // auxController.x().whileTrue(
        // drivetrain.applyRequest(()->
        //     robotDrive.withVelocityX(vision.getOutputX())
        //         .withVelocityY(vision.getOutputY())
        //         .withRotationalRate(vision.getOutputRot())));
    //CLIMB
    //(NOT USED FOR WAKE COMP)
        // auxController.povUp().whileTrue(Climby.manualClimb(0.1));
        // auxController.povDown().whileTrue(Climby.manualClimb(-0.1));
        // //joystick.b().onTrue(Climby.climbUp().andThen(Climby.climbUp()).andThen(Climby.climbUp()));
        // buttonBoard.button(1).onTrue(Climby.climbUp());
        // buttonBoard.button(2).onTrue(Climby.climbUp().andThen(Climby.climbUp()));
        // buttonBoard.button(3).onTrue(Climby.climbUp().andThen(Climby.climbUp()).andThen(Climby.climbUp()));
        // buttonBoard.button(0).onTrue(Climby.climbDown());
}

    //DASHBOARD

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
        drivetrain.registerTelemetry(logger::telemeterize);}
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        /*final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );*/
        return autoChooser.getSelected();
        
        // return drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     ).withTimeout(2)
        //     .andThen(intakeTilt.toggleRotate()
        //             .alongWith(shooterSub.autoShootCommand()
        //             .alongWith(intermediate.autoSpinCommand())));
        //intakeTilt.toggleRotate().alongWith(shooterSub.autoShootCommand().alongWith(intermediate.autoSpinCommand()));
        //return Commands.print("No auto enabled");
    }
    
}
