// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class shooter extends SubsystemBase {
  public TalonFX topLeftMotor = new TalonFX(19);
  public TalonFX topRightMotor = new TalonFX(17);
  public TalonFX BottomLeftMotor = new TalonFX(20);
  public TalonFX BottomRightMotor = new TalonFX(18);
  public CurrentLimitsConfigs flyWheelCurrentLimits = new CurrentLimitsConfigs();
  public CurrentLimitsConfigs RollerCurrentLimits = new CurrentLimitsConfigs();

  public PIDController softStartPID = new PIDController(1, 0, 0);

  public TalonFX rollerMotor = new TalonFX(21);
  //the three motors, named based on hight
  // fist distance is enum name second is verable name
  public static double position;

  /** Creates a new shooter. */
  public shooter() {
    topLeftMotor.setNeutralMode(NeutralModeValue.Coast);
    topRightMotor.setNeutralMode(NeutralModeValue.Coast);
    BottomLeftMotor.setNeutralMode(NeutralModeValue.Coast);
    BottomLeftMotor.setNeutralMode(NeutralModeValue.Coast);
    rollerMotor.setNeutralMode(NeutralModeValue.Coast);
    flyWheelCurrentLimits.SupplyCurrentLimitEnable = true; 
    flyWheelCurrentLimits.SupplyCurrentLimit = 20.0;
    RollerCurrentLimits.SupplyCurrentLimitEnable = true;
    RollerCurrentLimits.SupplyCurrentLimit = 20.0;
    topLeftMotor.getConfigurator().apply(flyWheelCurrentLimits);
    topRightMotor.getConfigurator().apply(flyWheelCurrentLimits);
    BottomLeftMotor.getConfigurator().apply(flyWheelCurrentLimits);
    BottomLeftMotor.getConfigurator().apply(flyWheelCurrentLimits);
    rollerMotor.getConfigurator().apply(RollerCurrentLimits);
    
    setDefaultCommand(run(() ->{
      setShooterSpeed(0);
      rollerMotor.set(0);
    }));
  }

  public double getShooterSpeed(){
    return topLeftMotor.get();
  }

  public double getShooterOutput(double speed){
    return softStartPID.calculate(getShooterSpeed(), speed);
  }

  public void setShooterSpeed(double speed){
    topLeftMotor.set(-speed);
    topRightMotor.set(speed);
    BottomLeftMotor.set(-speed);
    BottomRightMotor.set(speed);
  }

  public Command runShooterWheels(double speed){
    return run(()->{
      setShooterSpeed(speed);
    });
  }

  public Command staticShoot(double rollerSpeed, double flyWheelSpeed){
    return run(()->{
      rollerMotor.set(-rollerSpeed);
      setShooterSpeed(flyWheelSpeed);
    });
  }

  public Command autoShootCommand(){
    return run(()->{
      setShooterSpeed(0.7);
      rollerMotor.set(-1);
    }).withTimeout(12).andThen(runOnce(()->{
      setShooterSpeed(0);
      rollerMotor.set(0);
    }));
  }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
