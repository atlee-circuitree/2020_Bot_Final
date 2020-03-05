/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballObstructionSensorSubsystem;
import frc.robot.subsystems.elevatorMotorSubsystem;
import frc.robot.subsystems.wheelMotorSubsystem;


public class runUntilObstructedSensorCommand extends CommandBase {

    ballObstructionSensorSubsystem m_subsystem;

    public runUntilObstructedSensorCommand(ballObstructionSensorSubsystem sensorSubsystem) {
     
    super();
    m_subsystem = sensorSubsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_subsystem.isObstructed() == true);

       end(true);
  
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

     

  }

}
