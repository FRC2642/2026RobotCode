// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakeTilt.RotationPositions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
public class IntakeSpin extends SubsystemBase {
    public Boolean isSpinning = true;
  public TalonFX spinMotor = new TalonFX(0);
  public Command decideSpin(Boolean isSpinning){
    if (isSpinning ==true) {
      return new RunCommand(()->{
        spinToggle(0,false);
      });
    } else {
      return new RunCommand(()->{
        spinToggle(1,true);
    });
    } 
  }
  public Command spinToggle(double speed, Boolean newState){
    return new RunCommand(()->{
      isSpinning = newState;
      spinMotor.set((speed));
    });
  }
  @Override
  public void periodic() {}
}
