// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class Vision extends SubsystemBase {
public double[] measuments = {0,0,0,0,0,0};
public PIDController rotPID = new PIDController(0.2, 0, 0);
public int tagID;
public RawFiducial[] fiducials;
  public Vision() {
  }

  public Command print(){
    return run(()->{
      //System.out.println("x: " + LimelightHelpers.getTX(""));
      //System.out.println("y: " + LimelightHelpers.getTY(""));
      // for (double data: LimelightHelpers.getBotPose_TargetSpace("")){
      //   System.out.println(data);
      // }
      if (LimelightHelpers.getTV("")){
        updateMeasurments();
        // System.out.println("x: " + measuments[2]);
        // System.out.println("y: " + measuments[0]);
        // System.out.println("rot: " + measuments[4]);
        System.out.println("distance: " + getDistance());
      }

    });
  }

  public double getRotateOutput(){
    double output = rotPID.calculate(LimelightHelpers.getTX(""), 0);
    if (output > 1){
      output = 1;
    }
    if (output < -1){
      output = -1;
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

  public void updateMeasurments(){
    measuments = LimelightHelpers.getBotPose_TargetSpace("");

    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}