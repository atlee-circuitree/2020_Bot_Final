/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LidarSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;


public class CenterOnTargetLimelight extends CommandBase {
  DrivetrainSubsystem s_DriveTrainSubsystem;
  LimeLightSubsystem s_LimeLightSubsystem;

  public CenterOnTargetLimelight(DrivetrainSubsystem driveSubsystem, LimeLightSubsystem LimeLight) {
    super();
    s_DriveTrainSubsystem = driveSubsystem; 
    s_LimeLightSubsystem = LimeLight;
    addRequirements(s_LimeLightSubsystem, s_DriveTrainSubsystem);
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
