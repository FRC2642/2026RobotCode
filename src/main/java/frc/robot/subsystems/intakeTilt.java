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
  public TalonFX tiltA = new TalonFX(0);
  public TalonFX tiltB = new TalonFX(1);
  public DutyCycleEncoder encoder = new DutyCycleEncoder(0);
  public PIDController PID = new PIDController(1,0,0);

  public Trigger positionReached = new Trigger(() -> Math.abs(getEncoderValue() - motorState.position) < 0.01);
  public double getEncoderValue(){
    return encoder.get();
  }
  /** Creates a new intakeTilt. */
  public intakeTilt() {
    tiltA.setNeutralMode(NeutralModeValue.Brake);
    tiltB.setNeutralMode(NeutralModeValue.Brake);
    setDefaultCommand(runOnce(()->{
      tiltA.set(0);
      tiltB.set(0);
    }));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
