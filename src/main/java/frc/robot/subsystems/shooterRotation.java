package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intakeTilt.RotationPositions;

public class shooterRotation extends SubsystemBase {
public TalonFX leftTopMotor = new TalonFX(24);
public TalonFX rightTopMotor = new TalonFX(0);
public double maxRotateSpeed = 0.8;
public shooterRotationEnum motorState = shooterRotationEnum.standby;
public DutyCycleEncoder encoder = new DutyCycleEncoder(0);
public PIDController PID = new PIDController(1,0,0);
public Trigger positionReached = new Trigger(() -> Math.abs(getEncoderValue() - motorState.position) < 0.1);


public enum shooterRotationEnum {
standby(1),
halfway(0),
full(0);

public final double position;
shooterRotationEnum(double pos){
   position= pos;
}
}
public double getEncoderValue(){
    return encoder.get();
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

  public Command rotate1(shooterRotationEnum newState){
    return runOnce(()->{
    rotate1(shooterRotationEnum.standby);
    });};
public Command rotate2(shooterRotationEnum newestate){
    return runOnce(()->{
        rotate2(shooterRotationEnum.halfway);
    });
};
public Command rotate3(shooterRotationEnum newestate){
    return runOnce(()->{
        rotate3(shooterRotationEnum.full);
    });
}
    @Override
  public void periodic() {
  }};