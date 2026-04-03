// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class Dashboard extends SubsystemBase {
  public PowerDistribution powerDistribution = new PowerDistribution();
  public Vision visionSub;
  public intakeTilt IntakeTilt;
  public CommandXboxController controller;
  public Dashboard(Vision vision, CommandXboxController controller, intakeTilt intake) {
    this.controller = controller;
    this.IntakeTilt = intake;
    this.visionSub = vision;
    setDefaultCommand(sendAllData());
  }

  public Command sendAllData(){
    return run(()->{
    //Power Distribution
      SmartDashboard.putData("PDP", powerDistribution);
    //Robot Speed
      SmartDashboard.putNumber("Swerve Speed", getSwerveSpeed());
    //Intake Tilt
      SmartDashboard.putNumber("Intake Tilt", IntakeTilt.getEncoderValue());
    });
  }
  public double getSwerveSpeed(){
        double xVelocity = Math.abs(-controller.getLeftY() * Climby.constrain(controller.getLeftTriggerAxis()+0.5, 0 ,1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
        double yVelocity = Math.abs(-controller.getLeftX() * Climby.constrain(controller.getLeftTriggerAxis()+0.5, 0 ,1) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
        
        return (xVelocity + yVelocity)/2;
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
