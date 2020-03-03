/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooterIntakeSubsystem;
import frc.robot.subsystems.shooterMotorSubsystem;


public class ShootWaitVelocity extends CommandBase {

  shooterIntakeSubsystem m_subsystem;

  shooterMotorSubsystem m_subsystem2;
   
  public ShootWaitVelocity(shooterIntakeSubsystem intakeSubsystem, shooterMotorSubsystem shooterSubsystem) {
     
    super();
    m_subsystem = intakeSubsystem;
    m_subsystem2 = shooterSubsystem;
    addRequirements(m_subsystem, m_subsystem2);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    int velocity = m_subsystem2.getVelocity();
    System.out.println(velocity);
    if (Math.abs(velocity) < 500) {

      m_subsystem.conveyorOnly();

    } else {

      m_subsystem.stopConveyor();

    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_subsystem.stopConveyor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
