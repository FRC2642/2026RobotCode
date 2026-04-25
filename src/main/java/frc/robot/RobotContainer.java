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
                    
    //RESET GYRO
        //if somthing funky is happening its probrobly somthing to do with the gyro vision measuments
        //or robot pose.
        //I don't fully know how it works or how it might affect auto.
        //You might need to do some testing to flip between red and blue
        controller.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        //add new vision measument
        //more comments in the vision subsystem to maybe help
        controller.y().onTrue(vision.updatePose());
    //AUTO AIM
        //This works if the robot knows approx where it is on the field based on the most recent vision measuments
        //it can align for red side, not sure about blue side, this might be somthing you have to play around with on saturday (4/25)
        controller.b().whileTrue(
            AutoBuilder.pathfindToPose(
                new Pose2d(1.5, 4, Rotation2d.fromDegrees(0)), //Target Pose
                new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720)),
                0)
            );
    //SHOOT
        //Shoot once flywheel is up to speed, we can automate it later if we have time, but for now its manual
        auxController.leftTrigger().whileTrue(shooterSub
            .runShooterWheels(Constants.SHOOTER_ROLLER_1_SPEED, 
                                Constants.SHOOTER_ROLLER_2_SPEED, 
                                Constants.SHOOTER_FLYWHEEL_SPEED)
                .alongWith(intermediate.Spin(Constants.INTERMEDIATE_SPEED)));
    //START UP FLYWHEEL
        //It takes approx 2 seconds to get to full speed
        auxController.y().whileTrue(shooterSub
            .runShooterWheels(0, 0, 
                            Constants.START_FLYWHEEL_SPEED));
    //PASS
        //untested, currently this just sets it to max, takes approx 4 seconds to reach
        auxController.leftTrigger().whileTrue(shooterSub
            .runShooterWheels(Constants.SHOOTER_ROLLER_1_SPEED, 
                                Constants.SHOOTER_ROLLER_2_SPEED, 
                                Constants.PASSING_FLYWHEEL_SPEED)
                .alongWith(intermediate.Spin(Constants.INTERMEDIATE_SPEED)));
    //INTAKE TOGGLE
        auxController.a().onTrue(intakeTilt.toggleRotate());
    //INTAKE PULSE
        //I want to work on this, perhaps monday(4/25)?
        //Low priority
        //auxController.x().whileTrue(intakeTilt.Pulse());
    //INTAKE SPIN
        auxController.b().whileTrue(intakeSpin.spin(Constants.INTAKE_SPIN_SPEED));
    //MANUAL INTAKE TILT
        //UP
        auxController.rightBumper().whileTrue(intakeTilt.manualIntake(Constants.MANUAL_TILT_SPEED));
        //DOWN
        auxController.leftBumper().whileTrue(intakeTilt.manualIntake(-Constants.MANUAL_TILT_SPEED));
    //REVERSE REVERSE
        //This wont work correctly rn, its a quick fix tho. I'll do it on monday.
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
