// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

  public TalonFX rollerMotor = new TalonFX(21);
  //the three motors, named based on hight
  public shootModes shootMode;
  // fist distance is enum name second is verable name
  public static double position;
    // 
    //   public final double velocity(){
    //   //       gravity     dist from goal            goalY-startY                                        allat times this                                           
    //   return(((-9.81/2)*((Math.pow(getDistance, 2))/(1.828-0.4036-(Math.sin(shootAngle)/Math.cos(shootAngle))*distance)*(Math.pow(Math.cos(shootAngle), 2)))));
    //   // x is place holder thing */
    // }
  // public static double shootAngle = 0.436332;
  // public double topShooterSpeed = 0.00;
  // public double bottonShooterSpeed = 0.00;
  // public double wheelDiameter;
      // public double RPM = (velocity() / (Math.PI*wheelDiameter));
      //public double rotationSpeed = 1/RPM;

  /** Creates a new shooter. */
  public shooter() {
    topLeftMotor.setNeutralMode(NeutralModeValue.Brake);
    topRightMotor.setNeutralMode(NeutralModeValue.Brake);
    BottomLeftMotor.setNeutralMode(NeutralModeValue.Brake);
    BottomLeftMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotor.setNeutralMode(NeutralModeValue.Brake);
    flyWheelCurrentLimits.SupplyCurrentLimitEnable = true; 
    flyWheelCurrentLimits.SupplyCurrentLimit = 40.0;
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
  
  // public Command shoot(double Distance){
  //   return run(()-> {
  //     topMotor.set(1*Distance);
  //     intakeShooterMotor.set(-0.3*Distance);
  // //bottonMotor.set(0*bottonShooterSpeed);
  // // to use a enum I probley need a if statment but I don't know if we're going to be doing anything like that
  // //so I guess this will stay not completed. YAY.
  //   }).andThen(runOnce(()->{
  //     topMotor.set(0);
  //     intakeShooterMotor.set(0);
  //   }));
  // }

  // public Command intakeShoot(){
  //   return run(()->{
  //     intakeShooterMotor.set(1);
  //   }).andThen(runOnce(()->{
  //     intakeShooterMotor.set(0);
  //   }));
  // }

  public void setShooterSpeed(double speed){
    topLeftMotor.set(-speed);
    topRightMotor.set(speed);
    BottomLeftMotor.set(-speed);
    BottomRightMotor.set(speed);
  }
  public enum shootModes{
    somethingtoputhere(1);
    //the names are for the motors they corrolate to I was running out of ways to discribe top and bottom
    public final double position;
    shootModes(double pos){
      position = pos; 
    }
  }
public double getRPM(){
double ticksPer100ms = topLeftMotor.getVelocity().getValueAsDouble();
        double ticksPerRev = 2048.0; // Falcon 500 encoder resolution
        double rps = (ticksPer100ms * 10) / ticksPerRev;
        return rps * 60.0;
    } 

  public Command runShooterWheels(double speed){
    return run(()->{
      setShooterSpeed(speed);
    });
  }
  public Command staticShoot(double speed){
    return run(()->{
      rollerMotor.set(-speed);
      setShooterSpeed(0.7);
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
