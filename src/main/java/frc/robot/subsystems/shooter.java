// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
  public TalonFX topMotor = new TalonFX(0);
  public TalonFX bottonMotor = new TalonFX(0);
  public TalonFX intakeShooterMotor = new TalonFX(0);
  public Distance shooterSpeed;
  public static double position;
  public Distance sped = Distance.upper;
  public double topShooterSpeed = 0.00;
  public double bottonShooterSpeed = 0.00;
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
public Command shoot(Distance shooterspeed){
  return run(()-> {
topMotor.set(topShooterSpeed*0);
bottonMotor.set(0*bottonShooterSpeed);
// to use a enum I probley need a if statment but I don't know if we're going to be doing anything like that
//so I guess this will stay not completed. YAY.
  });
}
public Command intakeShoot(){
  return run(()->{
intakeShooterMotor.set(0);
  });
}
public enum Distance{

upper(1),
lowwer(0);
//the names are for the motors they corrolate to I was running out of ways to discribe top and bottom
public final double position;
Distance(double pos){
position = pos; 
}


  /*@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }*/
}}
