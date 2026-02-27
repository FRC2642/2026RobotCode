// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private final CommandJoystick buttonBoard = new CommandJoystick(2);
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandXboxController auxController = new CommandXboxController(1);

    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Intermediate intermediate = new Intermediate();
    public final Vision vision = new Vision();
    public final intakeTilt intakeTilt = new intakeTilt();
    public final Dashboard dash = new Dashboard(vision);
    public final IntakeSpin intakeSpin = new IntakeSpin();
    public final shooter shooterSub = new shooter();
    public Climby Climby = new Climby();

    public RobotContainer() {
        configureBindings();
    }
    private void configureBindings() {
        //DEFAULT SWERVE
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-controller.getRightX() * MaxAngularRate))); // Drive counterclockwise with negative X (left)
        //RESET GYRO
        controller.rightTrigger().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        //AUTO AIM
        controller.leftBumper().whileTrue(
            drivetrain.applyRequest(()->
            drive.withVelocityX(-controller.getLeftY() * MaxSpeed)
                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                .withRotationalRate(vision.getRotateOutput())));
        //SHOOT
        controller.rightBumper().whileTrue(shooterSub.shoot(1));
        //INTERMEDIATE
        controller.x().whileTrue(intermediate.Spin(1));
        controller.y().whileTrue(intermediate.Spin(-1));
        auxController.leftTrigger().whileTrue(intermediate.Spin(-1 * auxController.getLeftTriggerAxis()));
        auxController.rightTrigger().whileTrue(intermediate.Spin(1 * auxController.getRightTriggerAxis()));
        //INTAKE TOGGLE
        auxController.a().onTrue(intakeTilt.rotate(intakeTilt.motorState));
        auxController.b().whileTrue(intakeSpin.spin());
        //auxController.a().onTrue(intakeTilt.decideRotation(intakeTilt.motorState));
        //auxController.a().onTrue(intakeSpin.decideSpin(intakeSpin.isSpinning));
        //MANUAL INTAKE TILT
        controller.a().whileTrue(intakeTilt.manualIntake(0.1));
        controller.b().whileTrue(intakeTilt.manualIntake(-0.1));

        //climb
        //joystick.b().onTrue(Climby.climbUp().andThen(Climby.climbUp()).andThen(Climby.climbUp()));
        buttonBoard.button(1).onTrue(Climby.climbUp());
        buttonBoard.button(2).onTrue(Climby.climbUp().andThen(Climby.climbUp()));
        buttonBoard.button(3).onTrue(Climby.climbUp().andThen(Climby.climbUp()).andThen(Climby.climbUp()));
        buttonBoard.button(0).onTrue(Climby.climbDown());



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
        final var idle = new SwerveRequest.Idle();
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
        );
    }
}
