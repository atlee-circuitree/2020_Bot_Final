package frc.robot.commands;


import frc.robot.subsystems.ballObstructionSensorSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ballObstructionSensorCommand extends CommandBase {

    ballObstructionSensorSubsystem m_subsystem;
     
    public ballObstructionSensorCommand(ballObstructionSensorSubsystem sensorSubsystem) {
       
      super();
      m_subsystem = sensorSubsystem;
      addRequirements(m_subsystem);
    }
@Override
public void execute() {

  m_subsystem.isObstructed();

}
}