package frc.robot.commands;

import frc.robot.subsystems.kickoutPnumaticSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

 
public class kickoutRobotAndRetractPnumaticCommand extends CommandBase {

    kickoutPnumaticSubsystem m_subsystem;

    public kickoutRobotAndRetractPnumaticCommand(kickoutPnumaticSubsystem pnumaticSubsystem) {

    super();
    m_subsystem = pnumaticSubsystem;
    addRequirements(m_subsystem);
    
  }

  @Override
  public void initialize() {

    m_subsystem.kickout();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  
     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

     m_subsystem.kickoutReverse();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }



}