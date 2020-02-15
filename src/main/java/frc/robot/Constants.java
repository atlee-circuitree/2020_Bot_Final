/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Map all the motors / pnumatics here

  //Drivetrain

  public static int driveFrontleftMotor = 2;
  public static int driveFrontrightMotor = 4;
  public static int driveBackleftMotor = 3;
  public static int driveBackrightMotor = 1;

  public static final int rotate90Left = -13;  
  public static final int rotate90Right = 13; 
  public static final int rotate180 = 26;      
  public static final double rotate235Left = -32.5;  
  public static final double rotate235Right = 32.5;
  public static final double allowedError = 0.05;
  public static int leftAxis = 0;
  public static int rightAxis = 0;

  //Compressors

  public static final int Compressor = 1;
 
  // Solenoids

  public static final int leftClimbPnumatic = 0;
  public static final int rightClimbPnumatic= 1;
  public static final int leftArmClimbPnumatic_Deploy = 2;
  public static final int rightArmClimbPnumatic = 3;
  public static final int shooterPnumatic = 4;
  public static final int kickoutPnumatic = 5;

  public static final int climbPnumatic_Deploy = 0;
  public static final int climbPnumatic_Retract = 1;
  public static final int climbArmPnumatic_Deploy = 2;
  public static final int climbArmPnumatic_Retract = 3;
  public static final int shooterPnumatic_Deploy = 7;
  public static final int shooterPnumatic_Retract = 6;
  public static final int kickoutPnumatic_Deploy = 4;
  public static final int kickoutPnumatic_Retract = 5;

  //public int compressorPort = 0;
  //public int spikePort = 0;
  
  Compressor airCompressor;

  // Motors

  public static int conveyorbeltRight = 6;
  public static int conveyorbeltLeft = 8;

  //public static int shooterrightMotor = 0;
  //public static int shooterleftMotor = 0;

  public static int elevatorMotor = 5;
  
  public static int intakeMotor = 9;

  //public static int colorwheelmotor = 0;

  public static int barmotor = 7;

  public static int rightShooter = 11;

  public static int leftShooter = 12;

  //Limit Switches

  public static boolean Obstructor = true;
   
}
