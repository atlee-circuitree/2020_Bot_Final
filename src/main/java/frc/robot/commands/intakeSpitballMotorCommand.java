/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooterMotorSubsystem;


public class intakeSpitballMotorCommand extends CommandBase {

  shooterMotorSubsystem m_subsystem;
   
  public intakeSpitballMotorCommand(shooterMotorSubsystem motorSubsystem) {
     
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

    m_subsystem.spitoutballs();

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
