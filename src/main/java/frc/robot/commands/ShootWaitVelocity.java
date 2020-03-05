/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballObstructionSensorSubsystem;
import frc.robot.subsystems.shooterIntakeSubsystem;
import frc.robot.subsystems.shooterMotorSubsystem;

public class ShootWaitVelocity extends CommandBase {

  shooterIntakeSubsystem m_subsystem;

  shooterMotorSubsystem m_subsystem2;

  ballObstructionSensorSubsystem m_subsystem3;
   
  public ShootWaitVelocity (shooterIntakeSubsystem intakeSubsystem, shooterMotorSubsystem shooterSubsystem, ballObstructionSensorSubsystem obstructionSubsystem) {
     
    super();
    m_subsystem = intakeSubsystem;
    m_subsystem2 = shooterSubsystem;
    m_subsystem3 = obstructionSubsystem;
    addRequirements(m_subsystem, m_subsystem2);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    int velocity = m_subsystem2.getVelocityError();

    if ((Math.abs(velocity) < 300)){
      
        m_subsystem.conveyorOnly();
       
    } else {

        m_subsystem.stopConveyor();

    }

      
    /*int velocity = m_subsystem2.getVelocityError();
    System.out.println(velocity);
    while (m_subsystem3.isNotObstructed()) {
   
      m_subsystem.conveyorOnly();
      //System.out.print("Waiting for a ball ");
     
  
    }
    m_subsystem.stopConveyor();
    
    while ((Math.abs(velocity) > 300)){
      
      velocity = m_subsystem2.getVelocityError();
      //System.out.print("Waiting for the shooter to get to speed ");
      
      System.out.print(velocity);


      
    }
    while (m_subsystem3.isObstructed()) {
      
      m_subsystem.conveyorOnly();
      //System.out.print("Waiting for the ball to shoot ");
 
    }
    //System.out.println("The ball is shot");
    m_subsystem.stopConveyor();*/

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
