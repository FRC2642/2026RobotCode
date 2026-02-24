// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.subsystems.intakeTilt.RotationPositions;
import edu.wpi.first.math.controller.PIDController;
import org.ejml.equation.Variable;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

@SuppressWarnings("unused")

public class Climby extends SubsystemBase {
    /** Creates a new Climby. */

    //We need to decide what the value of these variables should be.
    public double targetHeight = 1000;
    public double currentTarget = 0;
    public double rotationSpeed = 40; 

    //I probably understand this now.
    public TalonFX climbMotor = new TalonFX(1);
    public Encoder climbEncoder = new Encoder(0,1);
    public PIDController PID = new PIDController(1, 0, 0);
    public RotationPositions motorState;
    
    //The two triggers.
    public Trigger positionReached = new Trigger(() -> Math.abs(getEncoderValue() - currentTarget) < 0.01);
    
  //I might know what this means.
  public Climby(){
    doAbsolutelyNothingForNoReasonBecauseWhyNot();
  }

  //This fuction climbs up until it reaches the Target Value, and then goes down to zero.
  public Command climbUp(){
    return new RunCommand(()->{
      //Do you know where the *up* function goes? That's right! It goes in the *up* hole.
     currentTarget = targetHeight;
     climbMotor.set(climbMotorOutput(currentTarget));
    }).until(positionReached).andThen(new RunCommand(()->{

      //Now it goes back down so that it can humble itself.
      currentTarget = 0;
      climbMotor.set(climbMotorOutput(currentTarget));
    })).until(positionReached);
  }

  public Command climbDown(){
    //You guessed it! The climb down function goes in the *up* hole.
    return new RunCommand(()->{
     setcurrentTarget(targetHeight);
     climbMotor.set(climbMotorOutput(targetHeight));
    }).until(positionReached);
  }
  
  public double getEncoderValue(){
    //I was told it was easier.
    return climbEncoder.get();
  }
  
  public double climbMotorOutput(double targetPosition){
    //Funny math thing that I don't understand.
    //Just trust them bro it works.
    return PID.calculate(targetPosition, getEncoderValue());
  }

  public Command setcurrentTarget(double set){
    //Make sure to subcribe, unalive that like button, and say bonjour to that notification bell.
    return new RunCommand(()->{
      currentTarget = set;
    });
  }
  
  public Command setClimbMotorSpeed(double speed){
    return new RunCommand(()->{
    climbMotor.set(speed);  
    });
  }

  //This is for whenever you want to use the value seventeen
  //but, like, you forgot how to write it in decimal so you just
  //write it in English.
  public Integer seventeen(){
    return (16 + 1);
  }

 //¿Por qué no?
  public Command doAbsolutelyNothingForNoReasonBecauseWhyNot(){
    return new RunCommand(()->{
    //does nothing
    //for when you don't feel like doing anything
    //Among Us
    //El cumpleaños de Abraham Lincoln es el 12 de febrero de 1809.
    });
  }

  public double getRotationSpeed(){
    return rotationSpeed;
  }
  
  //to constrain my power level and epicness
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