package frc.robot.tannersSubsystem;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class TannersClimberSubsystem extends SubsystemBase{

  TalonFX m_climb; 
  FeedbackConfigs climbFeedbackConfigs = new FeedbackConfigs();
  CurrentLimitsConfigs climbCurrentLimits = new CurrentLimitsConfigs();

  private final PositionVoltage m_request = new PositionVoltage(0);
  private final VoltageOut climberVoltageRequest = new VoltageOut(0);


    public TannersClimberSubsystem() {
        m_climb = new TalonFX(Ports.climberMotor, "CANivore6121"); 
    
    




      // Motion Profile Position
        var slot0Configs = new Slot0Configs();
    //Per https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html#position-control
    //Recommend no S,V,A values yet.
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 1; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    m_climb.getConfigurator().apply(slot0Configs);
  

    climbFeedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    
    m_climb.getConfigurator().apply(climbFeedbackConfigs);


    climbCurrentLimits.withStatorCurrentLimit(80); //TODO: Set Current Limit higher if necessary
    climbCurrentLimits.withStatorCurrentLimitEnable(true);

    m_climb.getConfigurator().apply(climbCurrentLimits);
  

  }


    @Override
  public void periodic() {

  }

  public void climbExtend() {

    m_climb.setVoltage(20);
  }



    public void climbRetract() {

      m_climb.setVoltage(-20);

 }
 public void climbStop() {

  m_climb.setVoltage(0);
}
 
 public double getPosition() {

    System.out.println("climb Position"+ m_climb.getRotorPosition());
    return m_climb.getRotorPosition().getValueAsDouble();
 }

  public void stop(){
  
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

}