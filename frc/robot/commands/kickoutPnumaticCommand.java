/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import frc.robot.subsystems.kickoutPnumaticSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

 
public class kickoutPnumaticCommand extends InstantCommand {
   
  kickoutPnumaticSubsystem m_subsystem;

  public kickoutPnumaticCommand() {

    super();
    addRequirements(m_subsystem);
    m_subsystem.kickoutSetup();
    
  }

  @Override
  public void initialize() {

    m_subsystem.kickout();
    
  }

}