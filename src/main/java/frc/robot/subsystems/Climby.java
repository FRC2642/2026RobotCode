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

@SuppressWarnings("unused")
public class Climby extends SubsystemBase {
    /** Creates a new Climby. */    
    int[] arrayValues = {300,350,400,650};
    public double digitalRotation = 0;
    public double rotationSpeed = 5;
    public double targetEnum = 0;
    public int arrayIndex = 0;
  
    public Climby(){
  double digitalRotation = 0;
  }

  //This fuction climbs up until it reaches the Enum Value
  public Command climbUp(){
    return new RunCommand(()->{
    if (upKey()) {
      //Finds the next targetEnum value
      arrayIndex = 0;
      while(!(arrayValues[arrayIndex]>getDigitalRotation())||arrayIndex>=arrayValues.length) {
        arrayIndex += 1;
      };
      targetEnum = arrayValues[arrayIndex];
      //Goes up until it reaches the next value
      while(!((getDigitalRotation()>=targetEnum)||downKey())){
      rotationSpeed += 0;
      setDigitalRotation(constrain(getDigitalRotation() + getRotationSpeed(),getDigitalRotation(),targetEnum));
      }
    }
    });
  }

  //This fuction climbs down until it reaches the Enum Value
  //It is the same as the one above except flipped.
  public Command climbDown(){
     return new RunCommand(()->{
    if (downKey()) {
      //Finds the next targetEnum value
      arrayIndex = arrayValues.length;
      while(!(arrayValues[arrayIndex]<getDigitalRotation())||arrayIndex<=1) {
        arrayIndex -= 1;
      };
      targetEnum = arrayValues[arrayIndex];
      //Goes down until it reaches the next value
      while(!((getDigitalRotation()<=targetEnum)||upKey())){
      rotationSpeed += 0;
      setDigitalRotation(constrain(getDigitalRotation() - getRotationSpeed(),getDigitalRotation(),targetEnum));
      }
    }
    });
  }

  public Command Climb(){
    return new RunCommand(()->{
    while(true) {
    climbUp();
    climbDown();
    }
    });
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
  
  //Should this be put into a different subsystem?
  //I still need to actually bind this to the controller.
  public Boolean upKey (){
    return(true);
  }
  public Boolean downKey (){
    return (false);
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

  //Is this using the function?
  {Climb();};

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