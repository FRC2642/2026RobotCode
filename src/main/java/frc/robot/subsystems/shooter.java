package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class shooter extends SubsystemBase {
  public TalonFX flyWheel1Motor = new TalonFX(Constants.FLYWHEEL_MOTOR_1);
  public TalonFX flyWheel2Motor = new TalonFX(Constants.FLYWHEEL_MOTOR_2);
  public TalonFX flyWheel4Motor = new TalonFX(Constants.FLYWHEEL_MOTOR_4);
  public TalonFX roller1Motor = new TalonFX(Constants.SHOOTER_ROLLER_MOTOR_1);
  public TalonFX roller2Motor = new TalonFX(Constants.SHOOTER_ROLLER_MOTOR_2);
  public CurrentLimitsConfigs flyWheelCurrentLimits = new CurrentLimitsConfigs();
  public CurrentLimitsConfigs RollerCurrentLimits = new CurrentLimitsConfigs();
  public TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration();
  final MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);

  /** Creates a new shooter. */
  public shooter() {
    flyWheelCurrentLimits.SupplyCurrentLimitEnable = true; 
    flyWheelCurrentLimits.SupplyCurrentLimit = Constants.FLYWHEEL_CURRENT_LIMIT;
    RollerCurrentLimits.SupplyCurrentLimitEnable = true;
    RollerCurrentLimits.SupplyCurrentLimit = Constants.SHOOTER_ROLLER_CURRENT_LIMIT;

    flywheelConfigs.Slot0.kS = 0.25;
    flywheelConfigs.Slot0.kV = 0.12;
    flywheelConfigs.Slot0.kA = 0.01;
    flywheelConfigs.Slot0.kP = 0.07;
    flywheelConfigs.Slot0.kI = 0;
    flywheelConfigs.Slot0.kD = 0;
    flywheelConfigs.MotionMagic.MotionMagicAcceleration = 45;


    flyWheel1Motor.setNeutralMode(NeutralModeValue.Coast);
    flyWheel2Motor.setNeutralMode(NeutralModeValue.Coast);
    flyWheel4Motor.setNeutralMode(NeutralModeValue.Coast);

    roller1Motor.setNeutralMode(NeutralModeValue.Coast);
    roller2Motor.setNeutralMode(NeutralModeValue.Coast);

    flyWheel1Motor.getConfigurator().apply(flyWheelCurrentLimits);
    flyWheel2Motor.getConfigurator().apply(flyWheelCurrentLimits);
    flyWheel4Motor.getConfigurator().apply(flyWheelCurrentLimits);

    flyWheel1Motor.getConfigurator().apply(flywheelConfigs);
    flyWheel2Motor.getConfigurator().apply(flywheelConfigs);
    flyWheel4Motor.getConfigurator().apply(flywheelConfigs);

    roller1Motor.getConfigurator().apply(RollerCurrentLimits);
    roller2Motor.getConfigurator().apply(RollerCurrentLimits);

    setDefaultCommand(run(() ->{
      setShooterSpeed(0,0,0);
    }));
  }

  public void setShooterSpeed(double roller1Speed, double roller2Speed, double flywheelSpeed){
    roller1Motor.set(-roller1Speed);
    roller2Motor.set(roller2Speed);
    if(flywheelSpeed == 0){
      flyWheel1Motor.setControl(new CoastOut());
      flyWheel2Motor.setControl(new CoastOut());
      flyWheel4Motor.setControl(new CoastOut());
    }
    else{
      flyWheel1Motor.setControl(m_request.withVelocity(flywheelSpeed));
      flyWheel2Motor.setControl(m_request.withVelocity(flywheelSpeed));
      flyWheel4Motor.setControl(m_request.withVelocity(-flywheelSpeed));
    }
  }
  public Command TestShooterMotors(double motor, double speed){
    return run(()->{
      if(motor == 1){
        flyWheel1Motor.set(speed);
      }
      if(motor == 2){
        flyWheel2Motor.set(speed);
      }
      if(motor == 3){
        flyWheel4Motor.set(speed);
      }
      if(motor == 4){
        roller1Motor.set(speed);
      }
      if(motor == 5){
        roller2Motor.set(speed);
      }
      if (motor == 6){
        flyWheel1Motor.set(speed);
        flyWheel2Motor.set(speed);
        flyWheel4Motor.set(-speed);
      }
    });
  }

  public Command runShooterWheels(double roller1Speed, double roller2Speed, double flyWheelSpeed){
    return run(()->{
      setShooterSpeed(roller1Speed, roller2Speed, flyWheelSpeed);
      
    });
  }

  public double calculateFlywheelSpeed(){
    //calculate flywheel speed based on distance function in vision
    return 0;
  }

  public Command dynamicShoot(){
    return run(()->{
      setShooterSpeed(
        Constants.SHOOTER_ROLLER_1_SPEED, 
        Constants.SHOOTER_ROLLER_2_SPEED, 
        calculateFlywheelSpeed());
    });
  }

@Override
  public void periodic() {
  }
}
