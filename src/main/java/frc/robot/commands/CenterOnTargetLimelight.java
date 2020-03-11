/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.LinearFilter;
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

  final double STEER_K = 0.02; // how hard to turn toward the target
  final double DRIVE_K = 0.00; // how hard to drive fwd toward the target
  final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall
  final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast
  final double MAX_ERROR = 1; //Maximum distance we can be off on X axis
  final double DEADBAND_MIN = 0.30;

  LinearFilter filterLinearError = LinearFilter.movingAverage(3);
  int iterations = 0;

  public CenterOnTargetLimelight(DrivetrainSubsystem driveSubsystem, LimeLightSubsystem LimeLight) {
    super();
    s_DriveTrainSubsystem = driveSubsystem;
    s_LimeLightSubsystem = LimeLight;
    addRequirements(s_LimeLightSubsystem, s_DriveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      s_LimeLightSubsystem.SetPipeline(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    iterations++;
    s_LimeLightSubsystem.EnableLED();
    s_LimeLightSubsystem.ReadNetworkTables();
    double m_LimelightDriveCommand = 0.0;
    double m_LimelightSteerCommand = 0.0;
    if (!s_LimeLightSubsystem.HasValidTarget()) {
      return;
    }

    double vertOffset = s_LimeLightSubsystem.VerticalOffset();
    double vertOffsetAverage = filterLinearError.calculate(vertOffset);

   

    double steer_cmd = vertOffset * STEER_K;

    // if(iterations > 3 && Math.abs(vertOffsetAverage - vertOffsetAverage) < 1)//we're not moving
    // {
    //   steer_cmd = 0.10; //try to push past deadband    
    //   System.out.println("limelight - stuck - increasing acceleration");
    // }
    if(Math.abs(steer_cmd) < DEADBAND_MIN)
    {
      System.out.println("limelight power below deadband");
      if(steer_cmd > 0)
      {
        steer_cmd = DEADBAND_MIN;
      }
      else
      {
        steer_cmd = -DEADBAND_MIN;
      }
        
    }

    m_LimelightSteerCommand = steer_cmd;



    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - s_LimeLightSubsystem.TargetArea()) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;

    System.out.print(" offset ");
    System.out.print(vertOffset);
    System.out.print(" steer ");
    System.out.print(m_LimelightSteerCommand);
    System.out.print(" drive ");
    System.out.println(m_LimelightDriveCommand);

    s_DriveTrainSubsystem.driveRobot(m_LimelightSteerCommand, m_LimelightDriveCommand); //invert steer command because it's inverted in "driveRobot" function
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_DriveTrainSubsystem.driveRobot(0d, 0d);
    s_LimeLightSubsystem.SetPipeline(0);
    //s_LimeLightSubsystem.DisableLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_LimeLightSubsystem.HasValidTarget()) {
      //TODO - make sure we don't exit too quickly here - ideally we should try to get closer
      if(Math.abs(s_LimeLightSubsystem.VerticalOffset()) < MAX_ERROR)
      {
        
        return true; //close enough
      }
    }
    return false;
  }
}
