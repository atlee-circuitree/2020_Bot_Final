/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballObstructionSensorSubsystem;
import frc.robot.subsystems.shooterIntakeSubsystem;



public class intakeTakeballMotorCommand extends CommandBase {

  shooterIntakeSubsystem m_subsystem;
  ballObstructionSensorSubsystem m_obstructionsubsystem;
  
   
  public intakeTakeballMotorCommand(shooterIntakeSubsystem intakeSubsystem, ballObstructionSensorSubsystem sensorSubsystem) {
     
    super();
    m_subsystem = intakeSubsystem;
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

   
    if (m_obstructionsubsystem.isObstructed()) {
    
      m_subsystem.stopintake();
    }
    else
    {
      m_subsystem.takeinballs();
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_subsystem.stopintake();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
