package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevatorMotorSubsystem;
import frc.robot.subsystems.levelerMotorSubsystem;
import frc.robot.subsystems.limelightSubsystem;
import frc.robot.subsystems.wheelMotorSubsystem;


public class limelightValuesCommand extends CommandBase {

  limelightSubsystem m_subsystem;
   
  public limelightValuesCommand(limelightSubsystem cameraSubsystem) {
     
    super();
    m_subsystem = cameraSubsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   // m_subsystem.Printvalues();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

}
