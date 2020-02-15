/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevatorMotorSubsystem;
import frc.robot.subsystems.levelerMotorSubsystem;
import frc.robot.subsystems.wheelMotorSubsystem;


public class levelerLeftMotorCommand extends CommandBase {

  levelerMotorSubsystem m_subsystem;
   
  public levelerLeftMotorCommand(levelerMotorSubsystem motorSubsystem) {
     
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

    m_subsystem.levelLeftMotor();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_subsystem.stopLevelMotor();

  }

}
