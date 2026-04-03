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
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    public final SendableChooser<Command> autoChooser;
    public RobotConfig config;
    public Supplier<Pose2d> pose = new Supplier<Pose2d>() {
        @Override
        public Pose2d get() {
            return drivetrain.getState().Pose;
        }   
    };
    public Consumer<Pose2d> resetPose = new Consumer<Pose2d>() {

        @Override
        public void accept(Pose2d t) {
            drivetrain.runOnce(drivetrain::seedFieldCentric);
        }
    };
    public Supplier<ChassisSpeeds> sppeeds = new Supplier<ChassisSpeeds>() {

        @Override
        public ChassisSpeeds get() {
            return drivetrain.getState().Speeds;
        }
    };
    public BiConsumer<ChassisSpeeds, DriveFeedforwards> output = new BiConsumer<ChassisSpeeds,DriveFeedforwards>() {

        @Override
        public void accept(ChassisSpeeds t, DriveFeedforwards u) {
            drivetrain.applyRequest(() ->
                robotDrive.withVelocityX(MaxSpeed / 1) // Drive forward with negative Y (forward)
                    .withVelocityY(MaxSpeed / 1) // Drive left with negative X (left)
                    .withRotationalRate(MaxAngularRate / 1));
        }
        
    };
    public PathFollowingController pathController = new PathFollowingController() {

        @Override
        public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
            return drivetrain.getState().Speeds;    
        }

        @Override
        public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
            drivetrain.runOnce(drivetrain::seedFieldCentric);    
        }

        @Override
        public boolean isHolonomic() {
            return true;
        }
        
    };
    public RobotContainer() {
        configureBindings();
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(
            pose, // Robot pose supplier
            resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            sppeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            output, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            pathController, // PPLTVController is the built in path following controller for differential drive trains
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            drivetrain // Reference to this subsystem to set requirements
    );

        
        //make the autos so they show up in the auto selector
        drivetrain.ConfigureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.setDefaultOption("Taxi Auto", new PathPlannerAuto("Taxi Auto"));
        
        
        //autoChooser.addOption("Shoot", new PathPlannerAuto("Shoot Auto"));
        //create named commands
        //these are all the commands to perform certain actions during auto
        //NamedCommands.registerCommand("shoot", shooterSub.staticShoot(.8, .7));
        //NamedCommands.registerCommand("intermediate", intermediate.Spin(.3));
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
        auxController.rightBumper().whileTrue(intakeTilt.manualIntake(0.1));
        auxController.leftBumper().whileTrue(intakeTilt.manualIntake(-0.1));
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
        //return new PathPlannerAuto("Taxi Auto");
        return autoChooser.getSelected();
        //return Commands.print("No auto enabled");
    }
    
}
