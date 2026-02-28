// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
public class IntakeSpin extends SubsystemBase {
  public Boolean isSpinning = false;
  public TalonFX spinMotor = new TalonFX(25);
  
  public IntakeSpin(){
    spinMotor.setNeutralMode(NeutralModeValue.Brake);
    setDefaultCommand(runOnce(()->{
      spinMotor.set(0);
    }));
  }

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
  public Command spin(){
    return run(()->{
      spinMotor.set(0.3);
    }).andThen(runOnce(()->{
      spinMotor.set(0);
    }));
  }
  public Command reverseSpin(){
    return run(()->{
      spinMotor.set(-0.3);
    }).andThen(runOnce(()->{
      spinMotor.set(0);
    }));
  }

  public Command manualSpin(double speed){
    return run(()->{
      spinMotor.set(speed);
    }).andThen(runOnce(()->{
      spinMotor.set(0);
    }));
  }
  @Override
  public void periodic() {}
}
