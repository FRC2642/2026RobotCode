// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class intakeTilt extends SubsystemBase {
  //defining both motors on the thing
  public TalonFX tiltMotor = new TalonFX(Constants.INTAKE_TILT_MOTOR);
  public DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.INTAKE_TILT_ENCODER);

  public double maxRotateSpeed = 1;

  public PIDController PID = new PIDController(6,0,0);
  public RotationPositions motorState = RotationPositions.up;
  public RotationPositions autoMotorState = RotationPositions.slightlyUp;
  public Trigger positionReached = new Trigger(() -> Math.abs(getEncoderValue() - motorState.position) < 0.01);

  //SUBSYSTEM METHOD
  public intakeTilt() {
    tiltMotor.setNeutralMode(NeutralModeValue.Brake);
    setDefaultCommand(runOnce(()->{
      //System.out.println("tilt encoder: "+ getEncoderValue());
      tiltMotor.set(0);
    }));
  }
  public enum RotationPositions{
    //ADJUSTED DO NOT USE DIRECT ENCODER VALUE
    up(0.32), //
    down(0.65),
    pulseUp(0.31),
    pulseDown(0.41),
    slightlyUp(0.55);

    public final double position;
    RotationPositions(double pos){
      position = pos;
    }
  }
  public double getEncoderValue(){
    //ADJUSTED DO NOT USE DIRECT ENCODER VALUE
    double value = encoder.get();
    if (encoder.get() < 0.30){
      value = value + 0.7;
    }
    else{
      if(encoder.get() > 0.30){
        value = value - 0.30;
      }
    }
    return value;
  }
  public double getRotateOutput(){
    double output = PID.calculate(getEncoderValue(), motorState.position);
    if (output > maxRotateSpeed){
      output = maxRotateSpeed;
    }
    
    if (output < -maxRotateSpeed){
      output = -maxRotateSpeed;
    }
    return output;
  }

  public Command toggleRotate(){
    return runOnce(()->{
      System.out.println("toggled");
      if(motorState == RotationPositions.up){
        motorState = RotationPositions.down;
      }
      else{
        if(motorState == RotationPositions.down){
          motorState = RotationPositions.up;
      }}
    }).andThen(run(()->{
        System.out.println("rotating");
        tiltMotor.set(-getRotateOutput());
    })).until(positionReached);
  }
  // public Command ballsIn() {
  //   return run(()->{
  //     autoMotorState = RotationPositions.slightlyUp;
  //     System.out.println("rotating");
  //       tiltMotor.set(-getRotateOutput());
  //   }).until(positionReached).andThen(runOnce(()->{
  //     motorState = RotationPositions.down;
  //   }));
  // }
  public Command manualIntake(double speed){
    return run(()->{
      tiltMotor.set(speed);
    });
  }
  public Command autoTimeout() {
    return run(()->{
      tiltMotor.set(.3);
    })
    .withTimeout(1.5);
  }
  public Command Pulse(){
    return run(()->{
      if(getEncoderValue() >= 0.75){
        tiltMotor.set(0.5);
        System.out.println("pulsing up");
      }
      if(getEncoderValue() <= 0.65){
        tiltMotor.set(-0.5);
        System.out.println("pulsing down");
      }
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
