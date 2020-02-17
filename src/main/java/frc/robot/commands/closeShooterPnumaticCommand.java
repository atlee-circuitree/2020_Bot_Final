/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package src.main.java.frc.robot.commands;


import src.main.java.frc.robot.subsystems.shooterPnumaticSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;

 
public class closeShooterPnumaticCommand extends InstantCommand {
   
  shooterPnumaticSubsystem m_subsystem;

  public closeShooterPnumaticCommand(shooterPnumaticSubsystem pnumaticSubsystem) {
   
    super();
    m_subsystem = pnumaticSubsystem;
    addRequirements(m_subsystem);
    
  }

  @Override
  public void initialize() {

    m_subsystem.closeShooter();
    
  }

}