package frc.robot.commands;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class driveStraightUntilEncoderValue extends CommandBase {

  int encoderTarget;
  double encoderReadingLeft;
  double encoderReadingRight;
  DrivetrainSubsystem m_subsystem;

  public driveStraightUntilEncoderValue(int targetValue, DrivetrainSubsystem driveSubsystem) {

    m_subsystem = driveSubsystem;
    addRequirements(m_subsystem);
    encoderTarget = targetValue;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    encoderReadingLeft = m_subsystem.getLeftEncoder();
    encoderReadingRight = m_subsystem.getRightEncoder();
     
    if(encoderReadingLeft > encoderReadingRight) {

      m_subsystem.correctLeft();

    } else if (encoderReadingRight > encoderReadingLeft) {

      m_subsystem.correctRight();

    } else {

      m_subsystem.driveForwards();

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_subsystem.driveStop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (encoderReadingLeft >= encoderTarget && encoderReadingRight >= encoderTarget) {

    return true;

   } else {

    return false;

   }

  }

}
