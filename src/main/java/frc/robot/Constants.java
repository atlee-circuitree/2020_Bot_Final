/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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

  public static int PORT_OBSTRUCTION_SENSOR = 0;

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

  public static final int climbArmPnumatic = 2;
  public static final int climbHookLeftPnumatic = 3;
  public static final int climbHookRightPnumatic = 1;
  public static final int shooterAndKickoutPnumatic = 4;

  public static final int climbArmPnumatic_Deploy = 2;
  public static final int climbArmPnumatic_Retract = 3;
  public static final int climbHookLeftPnumatic_Deploy = 0;
  public static final int climbHookLeftPnumatic_Retract = 1;
  public static final int climbHookRightPnumatic_Deploy = 4;
  public static final int climbHookRightPnumatic_Retract = 5;
  public static final int shooterAndKickoutPnumatic_Deploy = 7;
  public static final int shooterAndKickoutPnumatic_Retract = 6;

  //public int compressorPort = 0;
  //public int spikePort = 0;
  
  Compressor airCompressor;

  // Motors

  public static int conveyorbeltRight = 6;
  public static int conveyorbeltLeft = 8;

  //public static int shooterrightMotor = 0;
  //public static int shooterleftMotor = 0;

  //Inverted

  public static int elevatorMotor = 9;
  
  public static int intakeMotor = 5;

  public static final int intakeEncoderPortA = 5;
  public static final int intakeEncoderPortB = 6;

  public static int wheelMotor = 10;

  //public static int colorwheelmotor = 0;

  public static int barmotor = 7;

  public static int rightShooter = 11;

  public static int leftShooter = 12;

  //Limit Switches

  public static boolean Obstructor = true;

  //Shooter Init

  public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
  public static final int kTimeoutMs = 30;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 7200 represents Velocity units at 100% output
     * 
	 * 	                                    			  kP   kI   kD   kF          Iz    PeakOut */
  public final static Gains kGains_Velocit = new Gains( 0.03, 0, 0, 0.048,  300,  1.00);

  // Trajectory values

  public static final double ksVolts = 0.191;
  public static final double kvVoltSecondsPerMeter = 0.0536;
  public static final double kaVoltSecondsSquaredPerMeter = 0.0136;

  // Correct Values
  public static final double kPDriveVel = 0.653;

  public static final double kTrackwidthMeters = 0.503174350774;
  

  // Sample code values
  public static final double kMaxSpeedMetersPerSecond = 1;
  public static final double kMaxAccelerationMetersPerSecondSquared = 1;

  // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  public static final double kEncoderDistancePerPulse = 0.00141389064; //double check this - and break out into a formula 

  public static final double kDistancePerWheelRevolutionMeters = 8 * 0.0254 * Math.PI; //8 inches, converted to meters 
  public static final double kGearReduction = 10.71;

  public static final boolean kGyroReversed = true;

  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);


//Need to supply real values Panten 3/6/2020
  public static final int[] kLeftEncoderPorts = new int[]{0, 1};
    public static final int[] kRightEncoderPorts = new int[]{2, 3};
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

}
