// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class shooter extends SubsystemBase {
  public TalonFX topMotor = new TalonFX(24);
  public TalonFX bottonMotor = new TalonFX(0);
  public TalonFX intakeShooterMotor = new TalonFX(0);
  //the three motors, named based on hight
  public shootModes distance;
  // fist distance is enum name second is verable name
  public static double position;
  // 
//   public final double velocity(){
//   //       gravity     dist from goal            goalY-startY                                        allat times this                                           
//   return(((-9.81/2)*((Math.pow(getDistance, 2))/(1.828-0.4036-(Math.sin(shootAngle)/Math.cos(shootAngle))*distance)*(Math.pow(Math.cos(shootAngle), 2)))));
//   // x is place holder thing */
// }
  public static double shootAngle = 0.436332;
  public double topShooterSpeed = 0.00;
  public double bottonShooterSpeed = 0.00;
  public double wheelDiameter;
 // public double RPM = (velocity() / (Math.PI*wheelDiameter));
  //public double rotationSpeed = 1/RPM;
  /** Creates a new shooter. */
  public shooter() {
topMotor.setNeutralMode(NeutralModeValue.Brake);
bottonMotor.setNeutralMode(NeutralModeValue.Brake);
intakeShooterMotor.setNeutralMode(NeutralModeValue.Brake);
setDefaultCommand(run(() ->{
topMotor.set(0);
bottonMotor.set(0);
intakeShooterMotor.set(0);
}));
}
public Command shoot(double Distance){
  return run(()-> {
topMotor.set(1*Distance);
//bottonMotor.set(0*bottonShooterSpeed);
// to use a enum I probley need a if statment but I don't know if we're going to be doing anything like that
//so I guess this will stay not completed. YAY.
  }).andThen(runOnce(()->{
    topMotor.set(0);
  }));
}
public Command intakeShoot(){
  return run(()->{
intakeShooterMotor.set(0);
  });
}
public enum shootModes{

at2meters(1),
at8feet(0),
at10feet(0);
//the names are for the motors they corrolate to I was running out of ways to discribe top and bottom
public final double position;
shootModes(double pos){
position = pos; 
}


  /*@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }*/
}}
