// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intermediate extends SubsystemBase {
  public TalonFX SpinMotor = new TalonFX(15);
  public CurrentLimitsConfigs motorCurrentLimits = new CurrentLimitsConfigs();

  /** Creates a new Intermediate. */
  public Intermediate() {
    SpinMotor.setNeutralMode(NeutralModeValue.Brake);
    motorCurrentLimits.SupplyCurrentLimitEnable = true; 
    motorCurrentLimits.SupplyCurrentLimit = 20.0;
    SpinMotor.getConfigurator().apply(motorCurrentLimits);
    
    setDefaultCommand(runOnce(()->{
      SpinMotor.set(0);
    }));
  }
  public Command Spin(double speed){
    return run(()->{
      SpinMotor.set(speed);
    });
  }

  public Command autoSpinCommand(){
    return run(()->{
      SpinMotor.set(0.75);
    }).withTimeout(12).andThen(runOnce(()->{
      SpinMotor.set(0);
    }));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
