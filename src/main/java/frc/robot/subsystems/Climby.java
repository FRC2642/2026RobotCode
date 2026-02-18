// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import org.ejml.equation.Variable;
import org.ejml.ops.SortCoupledArray_F32;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;


@SuppressWarnings("unused")
public class Climby extends SubsystemBase {
    /** Creates a new Climby. */
    //We need to set the Enum Value
    public double targetHeight = 1000;
    public double digitalRotation = 0;
    public double rotationSpeed = 5;
    public double targetEnum = 0;
    public int arrayIndex = 0;
    public Trigger positionReached = new Trigger(() -> getDigitalRotation() < 0.01);

    public Climby(){
  double digitalRotation = 0;
  }

  //This fuction climbs up until it reaches the Target Value, and then goes down to zero.
  public Command climbUp(){
    return new RunCommand(()->{
      setDigitalRotation(targetHeight);
      while (getDigitalRotation() >= 0) {
        setDigitalRotation(getDigitalRotation() - getRotationSpeed());
      };
    }).until(positionReached);
  }
 
  //Do it just cuz.
  public double getDigitalRotation(){
    return digitalRotation;
  }

  public Command setDigitalRotation(double set){
    return new RunCommand(()->{
      digitalRotation = set;
    });
  }
  
  //This is for whenever you want to use the value seventeen
  //but, like, you forgot how to write it in decimal so you just
  //write it in Enlgish.
  public Integer seventeen(){
    return (16 + 1);
  }

  public double getRotationSpeed(){
    return rotationSpeed;
  }
  
  public double constrain (double value, double min, double max){
    if (max < min){
      return (constrain(value, max, min));
    }
    if (value >= max) {
      return (max);
    }
    if (value <= min) {
      return (min);
    }
    return (value);
  }

  //¿Por qué no?
  public Command nothing(){
    return new RunCommand(()->{
    //does nothing
    //Among Us
    //El cumpleaños de Abraham Lincoln es el 12 de febrero de 1809.
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}