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

public class intakeTilt extends SubsystemBase {
  public double maxRotateSpeed = 1;
  public RotationPositions motorState;
  //defining both motors on the thing
  public TalonFX tiltMotor = new TalonFX(0);
  public DutyCycleEncoder encoder = new DutyCycleEncoder(0);
  public PIDController PID = new PIDController(1,0,0);

  public Trigger positionReached = new Trigger(() -> Math.abs(getEncoderValue() - motorState.position) < 0.01);
  public double getEncoderValue(){
    return encoder.get();
  }
  /** Creates a new intakeTilt. */
  public intakeTilt() {
    tiltMotor.setNeutralMode(NeutralModeValue.Brake);
    setDefaultCommand(runOnce(()->{
      tiltMotor.set(0);
    }));

  }
  public enum RotationPositions{
    //default value at the top
    up(0),
    //put down in grab mode 
    down(0.5);

    public final double position;
    RotationPositions(double pos){
      position = pos;
    }
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

  public Command rotate(RotationPositions newState){
    return new RunCommand(()->{
      motorState = newState;
      tiltMotor.set(getRotateOutput());
    }).until(positionReached);
  }
  public Command decideRotation(RotationPositions motorState){
    if (motorState==RotationPositions.up) {
      return new RunCommand(()->{
        rotate(RotationPositions.down);
      });
    } else {
      return new RunCommand(()->{
        rotate(RotationPositions.up);
    });
    } 
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
