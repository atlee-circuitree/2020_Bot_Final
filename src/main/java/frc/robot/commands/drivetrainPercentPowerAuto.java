package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class drivetrainPercentPowerAuto extends CommandBase {
    double drivePowerValue;
    DrivetrainSubsystem m_subsystem;

    public drivetrainPercentPowerAuto(double DrivePower, DrivetrainSubsystem driveSubsystem) {
 
    drivePowerValue = DrivePower;
    m_subsystem = driveSubsystem;
    addRequirements(m_subsystem);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_subsystem.driveSetPercentAuto(-.3);    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_subsystem.driveStop();


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
