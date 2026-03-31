// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tannersSubsystem;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;

public class TannersShooterSubsystem extends SubsystemBase {
 TalonFX m_leftShooterMotor; 
 TalonFX m_rightShooterMotor;
 TalonFX m_centerShooterMotor;
 VoltageOut m_request = new VoltageOut(0);

  
  public TannersShooterSubsystem() {
    
    m_leftShooterMotor = new TalonFX(Ports.leftLauncherMotor, "CANivore6121");
    m_centerShooterMotor = new TalonFX(Ports.centerLauncherMotor, "CANivore6121");
    m_rightShooterMotor = new TalonFX(Ports.rightLauncherMotor, "CANivore6121");
   

    var slot0Configs = new Slot0Configs();
      slot0Configs.kS = 0.05; //.5 V outpot to overcome static friction
      slot0Configs.kV = 0.12; //A velocity target of 1 rps results in a 0.12 V output
      slot0Configs.kA = 0.01; //An acceleration of 1 rps/s requires 0.01 V output
      slot0Configs.kP = 0.1; 
      slot0Configs.kI = 0;
      slot0Configs.kD = 0;

     m_leftShooterMotor.getConfigurator().apply(slot0Configs);
     m_centerShooterMotor.getConfigurator().apply(slot0Configs);
     m_rightShooterMotor.getConfigurator().apply(slot0Configs);

  }

  @Override
  public void periodic() {
 
  }

  public void shoot(double speed) {

    final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    //Set speed in sooter robot container default shooter
    m_leftShooterMotor.setControl(m_request.withVelocity(speed));
    m_centerShooterMotor.setControl(m_request.withVelocity(speed));
    m_rightShooterMotor.setControl(m_request.withVelocity(speed));
 }
 
  

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  
}