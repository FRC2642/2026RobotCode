// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.equation.Variable;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;

import edu.wpi.first.math.controller.PIDController;

@SuppressWarnings("unused")
public class Climby extends SubsystemBase {
    /** Creates a new Climby. */

    //We need to decide what the value of these variables should be.
    public double targetHeight = 1000;
    public double digitalRotation = 0;
    public double rotationSpeed = 40;
    public double maxRotationSpeed = 50;

    //I lowkey don't know how this works.
    public TalonFX climbMotor = new TalonFX(1);
    public Encoder climbEncoder = new Encoder(0,1);
    public PIDController PID = new PIDController(1, 0, 0);
    
    ;

    public Trigger positionReached = new Trigger(() -> getDigitalRotation() < 0.01);
    
    //Yo guys what even is this doing here?
    public Climby(){
  double digitalRotation = 0;
  }

  //This fuction climbs up until it reaches the Target Value, and then goes down to zero.
  public Command climbUp(){
    return new RunCommand(()->{
      setDigitalRotation(targetHeight);
    }).until(positionReached).andThen(new RunCommand(()->{
       setDigitalRotation(getDigitalRotation() - getRotationSpeed());
    })).until(positionReached);
  }
 
  //Do it just cuz.
  public double getDigitalRotation(){
    return climbEncoder.get();
  }

  public Command setDigitalRotation(double set){
    return new RunCommand(()->{
      digitalRotation = set;
      climbMotor.set(set);
    });
  }
  
  //This is for whenever you want to use the value seventeen
  //but, like, you forgot how to write it in decimal so you just
  //write it in Enlgish.
  public Integer seventeen(){
    return (16 + 1);
  }

 //¿Por qué no?
  public Command doAbsolutelyNothingForNoReasonBecauseWhyNot(){
    return new RunCommand(()->{
    //does nothing
    //Among Us
    //El cumpleaños de Abraham Lincoln es el 12 de febrero de 1809.
    });
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
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}