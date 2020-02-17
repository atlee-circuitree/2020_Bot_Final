/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package src.main.java.frc.robot.commands;

import src.main.java.frc.robot.subsystems.climbPnumaticSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

 
public class climbarmdownPnumaticCommand extends InstantCommand {
   
  climbPnumaticSubsystem m_subsystem;

  public climbarmdownPnumaticCommand(climbPnumaticSubsystem pnumaticSubsystem) {

    super();
    m_subsystem = pnumaticSubsystem;
    addRequirements(m_subsystem);
    
  }

@Override
  public void initialize() {

    m_subsystem.climbArmDown();
    
  }

}