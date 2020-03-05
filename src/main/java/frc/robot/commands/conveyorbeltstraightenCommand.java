package frc.robot.commands;

import frc.robot.subsystems.ballObstructionSensorSubsystem;
import frc.robot.subsystems.shooterIntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooterMotorSubsystem;
 
 
public class conveyorbeltstraightenCommand extends CommandBase {

  shooterIntakeSubsystem m_subsystem;
 

  public conveyorbeltstraightenCommand(shooterIntakeSubsystem motorSubsystem) {
           
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
 
          m_subsystem.straightenballs();  

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
