package frc.robot.tannersCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.tannersSubsystem.TannersClimberSubsystem;

public class TannersClimberStop extends Command {
    TannersClimberSubsystem m_climb;

     
  

  public TannersClimberStop(TannersClimberSubsystem m_climb) {
    
    this.m_climb = m_climb;
  

    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  m_climb.climbStop();


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
