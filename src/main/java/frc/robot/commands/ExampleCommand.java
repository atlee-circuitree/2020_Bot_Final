/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package src.main.java.frc.robot.commands;

import src.main.java.frc.robot.subsystems.ExampleSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ExampleCommand extends CommandBase {
  /**
   * Creates a new ExampleCommand.
   *
   * @param m_kickoutPnumaticSubsystem The subsystem used by this command.
   */
  public ExampleCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  public ExampleCommand(ExampleSubsystem m_exampleSubsystem) {
}

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
