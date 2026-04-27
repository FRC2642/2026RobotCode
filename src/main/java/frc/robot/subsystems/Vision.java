// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class Vision extends SubsystemBase {
  public double[] measuments = {0,0,0,0,0,0};
  public PIDController rotPID = new PIDController(0.2, 0, 0);
  public PIDController drivePID = new PIDController(4, 0, 0);

  public PIDController climbXPID = new PIDController(2, 0, 0);
  public PIDController climbYPID = new PIDController(2, 0, 0);
  public PIDController climbRotPID = new PIDController(0.02, 0, 0);
  public int tagID;
  public double maxSpeed = 1;
  public RawFiducial[] fiducials;
  public CommandSwerveDrivetrain drivetrain;

  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    setDefaultCommand(run(()->{
    updateMeasurments();
    }));
  }

  public Command updatePose(){
    return runOnce(()->{
      //regardless of wether the robot is on red or blue side, the resetPose() function should use getBotPose2d_wpiBlue()
      //not the red one.
      //Changing this will not fix any inversion/flipping issues
      //technically it might fix it but it will mess up other stuff so try to find a different solution
      drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(""));
    });
  }
// FOR CLIMB ALLIGNMENT (NATE)
  public double getOutputX(){
    updateMeasurments();
    double output = climbXPID.calculate(measuments[2], -1.338);
    if (output > maxSpeed){
      output = maxSpeed;
    }
    if (output < -maxSpeed){
      output = -maxSpeed;
    }
    System.out.println("x: " + output);
    if(!LimelightHelpers.getTV("")){
      return 0;
    }
    return -output;
  }
  public double getOutputY(){
    updateMeasurments();
    double output = -climbYPID.calculate(measuments[0], 0.213);
    if (output > maxSpeed){
      output = maxSpeed;
    }
    if (output < -maxSpeed){
      output = -maxSpeed;
    }
    System.out.println("y: " + output);
    if(!LimelightHelpers.getTV("")){
      return 0;
    }
    return -output;
  }
  public double getOutputRot(){
    updateMeasurments();
    double output = climbRotPID.calculate(measuments[4], -2.2);
    System.out.println("rot: " + output);
    if(!LimelightHelpers.getTV("")){
      return 0;
    }
    return -output;
  }

  public double getRotateOutput(){
    double output = rotPID.calculate(LimelightHelpers.getTX(""), 0);
    if (output > 1){
      output = 1;
    }
    if (output < -1){
      output = -1;
    }
    return -output;
  }
  public double getDriveOutput(){
    double output = drivePID.calculate(getDistance(), 1);
    if (output < -1){
      output = -1;
    }
    if (output > 1){
      output = 1;
    }
    return output;
  }
  public double getHorizontalDriveOutput(){
    double output = rotPID.calculate(LimelightHelpers.getTX(""), 0);
    if (output < -1){
      output = -1;
    }
    if (output > 1){
      output = 1;
    }
    return output;
  }
  public double getDistance(){
    double distance = 0;
    if (LimelightHelpers.getTV("")){
      RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
      for (RawFiducial fiducial : fiducials) {
        tagID = fiducial.id;
        distance = fiducial.distToCamera;
      }
      return distance;
    }
    return 0;
  }

  public boolean inHubRange(){
    if (getDistance() <= 20 && getDistance() >= 0.5){
      return true;
    }
    else{
      return false;
    }
  }

  public void updateMeasurments(){
    if(LimelightHelpers.getTV("")){
      measuments = LimelightHelpers.getBotPose_TargetSpace("");
    }
    }
  
  
    @Override
  public void periodic() {
    // if(LimelightHelpers.getTV("")){
    //   drivetrain.resetPose(LimelightHelpers.getBotPose2d_wpiBlue(""));
    //   System.out.println("updating pose");
    //}
  }
}