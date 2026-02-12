// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class Climby extends SubsystemBase {
    public double digitalRotation = 0;
    /** Creates a new Climby. */
  public Climby() {
  double digitalRotation = 2;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
