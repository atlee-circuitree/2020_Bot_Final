package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ballObstructionSensorSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ballObstructionSensorCommand extends CommandBase {

    ballObstructionSensorSubsystem m_subsystem;
     
    public ballObstructionSensorCommand(ballObstructionSensorSubsystem sensorSubsystem) {
       
      super();
      m_subsystem = motorSubsystem;
      addRequirements(m_subsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
  
      m_subsystem.moveShooterDown();
  
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
  
      
  
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }