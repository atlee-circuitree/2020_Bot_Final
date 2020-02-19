package frc.robot.commands;
 
 
import frc.robot.subsystems.ballObstructionSensorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooterMotorSubsystem;
 
 
public class conveyorbeltObstructedCommand extends CommandBase {
 
        shooterMotorSubsystem m_subsystem;
        ballObstructionSensorSubsystem m_obstructionsubsystem;
        
         
        public conveyorbeltObstructedCommand(shooterMotorSubsystem motorSubsystem, ballObstructionSensorSubsystem sensorSubsystem) {
           
          super();
          m_subsystem = motorSubsystem;
          m_obstructionsubsystem = sensorSubsystem;
          addRequirements(m_subsystem, m_obstructionsubsystem);
      
        }
      
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
        }
      
        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
        
            if (m_obstructionsubsystem.isNotObstructed()) {
 
                
    
                m_subsystem.runShooter();
              }
        }
 
        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
      
      
        }
      
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
          return m_obstructionsubsystem.isObstructed();
        }
      }
