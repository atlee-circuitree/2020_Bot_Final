/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import frc.robot.subsystems.shooterPnumaticSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

 
public class closeShooterPnumaticCommand extends InstantCommand {
   
  shooterPnumaticSubsystem m_subsystem;

  public closeShooterPnumaticCommand() {

    super();
    addRequirements(m_subsystem);
    m_subsystem.shooter();
    
  }

  @Override
  public void initialize() {

    m_subsystem.closeShooter();
    
  }

}