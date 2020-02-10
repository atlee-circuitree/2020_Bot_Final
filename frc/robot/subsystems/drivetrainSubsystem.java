/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//Thanks to 2643 for us "borrowing" thier drive code
//https://github.com/2643/2020-Code/blob/in-progress/src/main/java/frc/robot/commands/Tankdrive.java

public class drivetrainSubsystem extends SubsystemBase {
  private static CANSparkMax leftFrontMotor = new CANSparkMax(Constants.driveFrontleftMotor, MotorType.kBrushless);
  private static CANSparkMax leftBackMotor = new CANSparkMax(Constants.driveBackleftMotor, MotorType.kBrushless);

  private static CANSparkMax rightFrontMotor = new CANSparkMax(Constants.driveFrontrightMotor, MotorType.kBrushless);
  private static CANSparkMax rightBackMotor = new CANSparkMax(Constants.driveBackrightMotor, MotorType.kBrushless);


  // Sets the PID and FF variables
 /* double kP = 0.00016;//0.006;
  double kI = 0;//0.000002;
  double kD = 0;//0.004;//0.2;
  double kFF = 0.000156;

  // Sets the max and min output for the motor speed
  double MaxOutput = 1;
  double MinOutput = -1;

  // Sets the max acceleration for the motors
  double maxAccel = 3000;
  int slotID = 0;
  int maxVel = 7000;
  int minVel = 0;

  double allowedErr = 0.1;

  /**
   * Creates a new Drivetrain.
   */
  
  public drivetrainSubsystem() {
    // Set the PID Controller for Left Front Motor
    /*leftFrontMotor.getPIDController().setP(kP, slotID);
    leftFrontMotor.getPIDController().setI(kI, slotID);
    leftFrontMotor.getPIDController().setD(kD, slotID);
    leftFrontMotor.getPIDController().setFF(kFF, slotID);
    leftFrontMotor.getPIDController().setOutputRange(MinOutput, MaxOutput, slotID);
    leftFrontMotor.getPIDController().setSmartMotionMaxAccel(maxAccel, slotID);
    leftFrontMotor.getPIDController().setSmartMotionAllowedClosedLoopError(allowedErr, slotID);
    leftFrontMotor.getPIDController().setSmartMotionMaxVelocity(maxVel, slotID);
    leftFrontMotor.getPIDController().setSmartMotionMinOutputVelocity(minVel, slotID);

    // Set the PID Controller for Left Back Motor
    leftBackMotor.getPIDController().setP(kP, slotID);
    leftBackMotor.getPIDController().setI(kI, slotID);
    leftBackMotor.getPIDController().setD(kD, slotID);
    leftBackMotor.getPIDController().setFF(kFF, slotID);
    leftBackMotor.getPIDController().setOutputRange(MinOutput, MaxOutput, slotID);
    leftBackMotor.getPIDController().setSmartMotionMaxAccel(maxAccel, slotID);
    leftBackMotor.getPIDController().setSmartMotionAllowedClosedLoopError(allowedErr, slotID);
    leftBackMotor.getPIDController().setSmartMotionMaxVelocity(maxVel, slotID);
    leftBackMotor.getPIDController().setSmartMotionMinOutputVelocity(minVel, slotID);

    // Set the PID Controller for Right Front Motor
    rightFrontMotor.restoreFactoryDefaults();
    rightFrontMotor.getPIDController().setP(kP, slotID);
    rightFrontMotor.getPIDController().setI(kI, slotID);
    rightFrontMotor.getPIDController().setD(kD, slotID);
    rightFrontMotor.getPIDController().setFF(kFF, slotID);
    rightFrontMotor.getPIDController().setOutputRange(MinOutput, MaxOutput, slotID);
    rightFrontMotor.getPIDController().setSmartMotionMaxAccel(maxAccel, slotID);
    rightFrontMotor.getPIDController().setSmartMotionAllowedClosedLoopError(allowedErr, slotID);
    rightFrontMotor.getPIDController().setSmartMotionMaxVelocity(maxVel, slotID);
    rightFrontMotor.getPIDController().setSmartMotionMinOutputVelocity(minVel, slotID);

    // Set the PID Controller for Right Back Motor
    rightBackMotor.restoreFactoryDefaults();
    rightBackMotor.getPIDController().setP(kP, slotID);
    rightBackMotor.getPIDController().setI(kI, slotID);
    rightBackMotor.getPIDController().setD(kD, slotID);
    rightBackMotor.getPIDController().setFF(kFF, slotID);
    rightBackMotor.getPIDController().setOutputRange(MinOutput, MaxOutput, slotID);
    rightBackMotor.getPIDController().setSmartMotionMaxAccel(maxAccel, slotID);
    rightBackMotor.getPIDController().setSmartMotionAllowedClosedLoopError(allowedErr, slotID);
    rightBackMotor.getPIDController().setSmartMotionMaxVelocity(maxVel, slotID);
    rightBackMotor.getPIDController().setSmartMotionMinOutputVelocity(minVel, slotID);
    
    // Sets the back motors to follow the front motors
    //leftBackMotor.follow(leftFrontMotor);
/*
    rightFrontMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    rightBackMotor.follow(rightFrontMotor);
*/
  }
  

  /**
   * Gets the encoder position of the left side of the drivetrain
   * 
   * @return double rotations
   */
  public double getLeftMotorEncoder() {
    return leftFrontMotor.getEncoder().getPosition();
  }

  /**
   * Gets the encoder position of the right side of the drivetrain
   * 
   * @return double rotations
   */
  public double getRightMotorEncoder() {
    return rightFrontMotor.getEncoder().getPosition();
  }

  /**
   * Sets the speed of the left side motors using Duty Cycle
   * 
   * @param speed from -1 to 1, speed to set motor
   */
  public void setLeftMotorSpeed(double speed) {
    leftFrontMotor.getPIDController().setReference(-speed, ControlType.kDutyCycle, 2, 0);
    leftBackMotor.getPIDController().setReference(-speed, ControlType.kDutyCycle, 3, 0);
  }

  /**
   * Sets the speed of the right side motors using Duty Cycle
   * 
   * @param speed -1 to 1, speed to set motor
   */
  public void setRightMotorSpeed(double speed) {
    rightFrontMotor.getPIDController().setReference(speed, ControlType.kDutyCycle, 1, 0);
    rightBackMotor.getPIDController().setReference(speed, ControlType.kDutyCycle, 4, 0);
  }

  /**
   * Sets the speed of the drivetrain using Duty Cycle
   * 
   * @param speed -1 to 1, speed to set motor
   */
  public void setMotorSpeed(double speed) {
    setLeftMotorSpeed(speed);
    setRightMotorSpeed(speed);
  }

  /**
   * Sets the position of the left side of the drivetrain using Position
   * 
   * @param rotations
   */
  //public void setLeftMotorPosition(double rotations) {
    //leftFrontMotor.getPIDController().setReference(rotations, ControlType.kSmartMotion, slotID, 0);
    //leftBackMotor.getPIDController().setReference(rotations, ControlType.kSmartMotion, slotID, 0);
  //}

  /**
   * Sets the position of the right side of the drivetrain using Position
   * 
   * @param rotations
   */
  //public void setRightMotorPosition(double rotations) {
    //rightFrontMotor.getPIDController().setReference(-rotations, ControlType.kSmartMotion, slotID, 0);
    //rightBackMotor.getPIDController().setReference(-rotations, ControlType.kSmartMotion, slotID, 0);
  //}

  //public void setAllMotorPosition(double rotations) {
    //setRightMotorPosition(rotations);
    //setLeftMotorPosition(rotations);
  //}

  /**
   * Resets the left encoder
   */
  public void resetLeftEncoder() {
    leftFrontMotor.getEncoder().setPosition(0);
    leftBackMotor.getEncoder().setPosition(0);
  }

  /**
   * Resets the right encoder
   */
  public void resetRightEncoder() {
    rightFrontMotor.getEncoder().setPosition(0);
    rightBackMotor.getEncoder().setPosition(0);
  }

  public void resetAllEncoder() {
    resetLeftEncoder();
    resetRightEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
