// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tannersCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.tannersSubsystem.TannersClimberSubsystem;

public class TannersClimberExtend extends Command {
    TannersClimberSubsystem m_climb;

     
  

  public TannersClimberExtend(TannersClimberSubsystem m_climb) {
    
    this.m_climb = m_climb;
  

    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  m_climb.climbExtend();


  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}