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


public class runUntilNotObstructedSensorCommand extends CommandBase {

    ballObstructionSensorSubsystem m_subsystem;

    public runUntilNotObstructedSensorCommand(ballObstructionSensorSubsystem sensorSubsystem) {
     
    super();
    m_subsystem = sensorSubsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  //run by the scheduler to check to see if the task is finished
  @Override
  public boolean isFinished() {
    return (m_subsystem.isObstructed() == false);
  }

}
