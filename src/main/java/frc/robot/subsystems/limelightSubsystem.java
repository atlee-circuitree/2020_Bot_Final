/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Limelight Values

 
public class limelightSubsystem extends SubsystemBase {

  //private static drivetrainSubsystem m_drivetrainSubsystem = new drivetrainSubsystem();

  boolean m_LimelightHasValidTarget = false;
  double m_LimelightDriveCommand = 0.0;
  double m_LimelightSteerCommand = 0.0;
  
  public limelightSubsystem() {
  
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); //Set LEDs to current pipeline setting
    
  }

  public void updateLimelightTracking() {

    

    final double STEER_K = 0.03; // how hard to turn toward the target
    final double DRIVE_K = 0.00; // how hard to drive fwd toward the target
    final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall
    final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    

    if (tv < 1.0) {
      m_LimelightHasValidTarget = false;
      m_LimelightDriveCommand = 0.0;
      m_LimelightSteerCommand = 0.0;
      return;
    }

    m_LimelightHasValidTarget = true;

    // Start with proportional steering

    double dead_band = 0.15;
    double steer_cmd = tx * STEER_K;
    if(steer_cmd != 0)
    {
      if(steer_cmd > 0 && steer_cmd < dead_band)
        steer_cmd =dead_band; //dead band
      else if(steer_cmd < 0 && steer_cmd > -dead_band)
        steer_cmd = -dead_band;
    }
    m_LimelightSteerCommand = steer_cmd;

    // try to drive forward until the target area reaches our desired area
    double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

    // don't let the robot drive too fast into the goal
    if (drive_cmd > MAX_DRIVE) {
      drive_cmd = MAX_DRIVE;
    }
    m_LimelightDriveCommand = drive_cmd;


    System.out.print("tv: ");
    System.out.print(tv);
    System.out.print(" tx ");
    System.out.print(tx);
    System.out.print(" ty ");
    System.out.print(ty);
    System.out.print(" ta ");
    System.out.print(ta);
    System.out.print(" steer ");
    System.out.print(m_LimelightSteerCommand);
    System.out.print(" drive ");
    System.out.print(m_LimelightDriveCommand);

  }

  public void LimelightSteer() {

    updateLimelightTracking();
 
    //if (m_LimelightHasValidTarget) {
    //    m_drivetrainSubsystem.robotDrive.arcadeDrive(m_LimelightDriveCommand, m_LimelightSteerCommand);
    //} else {
    //    m_drivetrainSubsystem.robotDrive.arcadeDrive(0.0, 0.0);
    //}
     
  }

  public void stopDrive() {

    //m_drivetrainSubsystem.robotDrive.arcadeDrive(0.0, 0.0);

  }

  @Override
  public void periodic() {
    //thor
    //tvert
    double thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    double tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
    SmartDashboard.putNumber("Limelight Horizontal", thor);
    SmartDashboard.putNumber("Limelight Vertical", tvert);
  }
 
}

