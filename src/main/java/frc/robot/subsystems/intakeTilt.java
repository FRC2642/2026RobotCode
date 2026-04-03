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

public class intakeTilt extends SubsystemBase {
  public double maxRotateSpeed = 1;
  public PIDController PID = new PIDController(3,0.1,0);

  public RotationPositions motorState = RotationPositions.up;

  //defining both motors on the thing
  public TalonFX tiltMotor = new TalonFX(14);
  public DutyCycleEncoder encoder = new DutyCycleEncoder(9);

  public Trigger positionReached = new Trigger(() -> Math.abs(getEncoderValue() - motorState.position) < 0.008);

  /** Creates a new intakeTilt. */
  public intakeTilt() {
    tiltMotor.setNeutralMode(NeutralModeValue.Brake);
    setDefaultCommand(runOnce(()->{
      System.out.println("tilt encoder: "+ getEncoderValue());
      tiltMotor.set(0);
    }));

  }
  public enum RotationPositions{
    //default value at the top
    //ADJUSTED DO NOT USE DIRECT ENCODER VALUE
    up(0.59),
    //put down in grab mode 
    down(0.945);

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
  public Command rotate(RotationPositions newState){
    return run(()->{
      motorState = newState;
      tiltMotor.set(-getRotateOutput());
      //System.out.println("going " + motorState + "|| Encoder: " + getEncoderValue());
    }).until(positionReached);
  }
  public Command manualIntake(double speed){
    return run(()->{
      tiltMotor.set(speed);
    });
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
