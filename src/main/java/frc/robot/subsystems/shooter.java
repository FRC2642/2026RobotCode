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
  public TalonFX flyWheel1Motor = new TalonFX(17);
  public TalonFX flyWheel2Motor = new TalonFX(20);
  //public TalonFX flyWheel3Motor = new TalonFX(22);
  public TalonFX flyWheel4Motor = new TalonFX(23);
  public TalonFX roller1Motor = new TalonFX(19);
  public TalonFX roller2Motor = new TalonFX(18);
  public CurrentLimitsConfigs flyWheelCurrentLimits = new CurrentLimitsConfigs();
  public CurrentLimitsConfigs RollerCurrentLimits = new CurrentLimitsConfigs();

  public PIDController softStartPID = new PIDController(1, 0, 0);

  //the three motors, named based on hight
  // fist distance is enum name second is verable name
  public static double position;

  /** Creates a new shooter. */
  public shooter() {
    flyWheel1Motor.setNeutralMode(NeutralModeValue.Coast);
    flyWheel2Motor.setNeutralMode(NeutralModeValue.Coast);
    //flyWheel3Motor.setNeutralMode(NeutralModeValue.Coast);
    flyWheel4Motor.setNeutralMode(NeutralModeValue.Coast);
    roller1Motor.setNeutralMode(NeutralModeValue.Coast);
    roller2Motor.setNeutralMode(NeutralModeValue.Coast);
    flyWheelCurrentLimits.SupplyCurrentLimitEnable = true; 
    flyWheelCurrentLimits.SupplyCurrentLimit = 25.0;
    RollerCurrentLimits.SupplyCurrentLimitEnable = true;
    RollerCurrentLimits.SupplyCurrentLimit = 25.0;
    flyWheel1Motor.getConfigurator().apply(flyWheelCurrentLimits);
    flyWheel2Motor.getConfigurator().apply(flyWheelCurrentLimits);
    //flyWheel3Motor.getConfigurator().apply(flyWheelCurrentLimits);
    flyWheel4Motor.getConfigurator().apply(flyWheelCurrentLimits);
    roller1Motor.getConfigurator().apply(RollerCurrentLimits);
    roller2Motor.getConfigurator().apply(RollerCurrentLimits);

    
    setDefaultCommand(run(() ->{
      setShooterSpeed(0,0,0);
    }));
  }

  public void setShooterSpeed(double roller1Speed, double roller2Speed, double flywheelSpeed){
    flyWheel1Motor.set(flywheelSpeed);
    flyWheel2Motor.set(flywheelSpeed);
    //flyWheel3Motor.set(flywheelSpeed);
    flyWheel4Motor.set(-flywheelSpeed);
    roller1Motor.set(-roller1Speed);
    roller2Motor.set(roller2Speed);
  }
  public Command TestShooterMotors(double motor, double speed){
    return run(()->{
      if(motor == 1){
        flyWheel1Motor.set(speed);
      }
      if(motor == 2){
        flyWheel2Motor.set(speed);
      }
      if(motor == 3){
        flyWheel4Motor.set(speed);
      }
      if(motor == 4){
        roller1Motor.set(speed);
      }
      if(motor == 5){
        roller2Motor.set(speed);
      }
    });
  }

  public Command runShooterWheels(double roller1Speed, double roller2Speed, double flyWheelSpeed){
    return run(()->{
      setShooterSpeed(roller1Speed, roller2Speed, flyWheelSpeed);
    });
  }

  public Command staticShoot(double rollerSpeed, double flyWheelSpeed){
    return run(()->{
      setShooterSpeed(flyWheelSpeed, flyWheelSpeed, flyWheelSpeed);
    });
  }

  public Command autoShootCommand(){
    return run(()->{
      setShooterSpeed(0.7, 0, 0);
    }).withTimeout(12).andThen(runOnce(()->{
      setShooterSpeed(0, 0, 0);
    }));
  }

@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
