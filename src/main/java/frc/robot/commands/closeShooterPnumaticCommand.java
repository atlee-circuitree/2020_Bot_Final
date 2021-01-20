/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import frc.robot.subsystems.shooterAndKickoutPnumaticSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

 
public class closeShooterPnumaticCommand extends InstantCommand {
   
  shooterAndKickoutPnumaticSubsystem m_subsystem;

  public closeShooterPnumaticCommand(shooterAndKickoutPnumaticSubsystem pnumaticSubsystem) {
   
    super();
    m_subsystem = pnumaticSubsystem;
    addRequirements(m_subsystem);
    
  }

  @Override
  public void initialize() {

    m_subsystem.closeShooter();
    
  }

}