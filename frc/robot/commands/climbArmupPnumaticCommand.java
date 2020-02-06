/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.climbArmPnumaticSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

 
public class climbArmupPnumaticCommand extends InstantCommand {
   
  climbArmPnumaticSubsystem m_subsystem;

  public climbArmupPnumaticCommand() {

    super();
    addRequirements(m_subsystem);
    m_subsystem.armClimb();
    
  }

  @Override
  public void initialize() {

    m_subsystem.climbUp();
    
  }

}