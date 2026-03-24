// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class shooterTilt extends SubsystemBase {
  public TalonFX tiltMotor = new TalonFX(22);
  public DutyCycleEncoder encoder = new DutyCycleEncoder(8);
  public PIDController pid = new PIDController(0.4, 0, 0.07);
  public tiltStates tiltState;
  public Trigger positionReached = new Trigger(() -> Math.abs(getEncoderValue() - tiltState.tilt) < 0.06);
  public double maxTiltSpeed = 1;


  public shooterTilt() {
    tiltMotor.setNeutralMode(NeutralModeValue.Brake);
    setDefaultCommand(runOnce(()->{
      tiltMotor.stopMotor();
    }));
  }

  public enum tiltStates{
    top(0.0),
    mid(0.12),
    bottom(0.25);
    public final double tilt;
    tiltStates(double pos){
      tilt = pos; 
    }
  }

  public double getEncoderValue(){
    return encoder.get();
  }

  public double getMotorOutput(){
    double output = pid.calculate(getEncoderValue(), tiltState.tilt);
    if (output > maxTiltSpeed){
      output = maxTiltSpeed;
    }
    if (output < -maxTiltSpeed){
      output = -maxTiltSpeed;
    }
    return output;
  }
  public Command tilt(tiltStates state){
    return run(()->{
      System.out.println("tilting: " + getEncoderValue());
      tiltState = state;
      tiltMotor.set(-getMotorOutput());
    }).until(positionReached).andThen(runOnce(()->{
      System.out.println("stopped tilting");
      tiltMotor.set(0);
    }));
  }

  public Command manualTilt(double speed){
    return new RunCommand(()->{
      System.out.println("shooter tilt encoder: " + getEncoderValue());
      tiltMotor.set(speed* 0.5);
    }).andThen(runOnce(()->{
      System.out.println("stopped");
      tiltMotor.set(0);
    }));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
