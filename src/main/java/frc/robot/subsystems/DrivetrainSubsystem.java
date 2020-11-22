/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// Added by Panten 3/6/2020
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; // Added by Panten 3/6/2020
//import edu.wpi.first.wpilibj.examples.ramsetecommand.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SPI;
//End Panten additions 3/6/2020

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.Constants;
//import frc.robot.RobotContainer;
import frc.robot.commands.drivetrainPercentPowerAuto;

public class DrivetrainSubsystem extends SubsystemBase {

    SpeedControllerGroup leftDrive;
    SpeedControllerGroup rightDrive;
    DifferentialDrive robotDrive;

    CANSparkMax left_frontmotor;
    CANSparkMax left_backmotor;
    CANSparkMax right_frontmotor;
    CANSparkMax right_backmotor;

    PIDController turnController;

    /**
     * Creates a new ExampleSubsystem.
     */
    
    public DrivetrainSubsystem() {
 
 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Added 3/6/2020 Panten
        // Update the odometry in the periodic block
        if (m_rightEncoder != null) // make sure setup has been called first
        {
            double leftPos = m_leftEncoder.getPosition();
            double rightPos = m_rightEncoder.getPosition();
        }
    }

    // The robot's drive
    DifferentialDrive m_drive;

    // The left-side drive encoder
    CANEncoder m_leftEncoder;

    // The right-side drive encoder
    CANEncoder m_rightEncoder;
     
    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    public void driveRobot(Double X, double Y) {

        robotDrive.arcadeDrive(-Y, X, true);
    }
    public void driveRobotLinear(double X, double Y) {
        robotDrive.arcadeDrive(-Y, X, false);
    }

    public void setBrakes()
    {
        left_frontmotor.setIdleMode(IdleMode.kBrake);
        left_backmotor.setIdleMode(IdleMode.kBrake);
        right_backmotor.setIdleMode(IdleMode.kBrake);
        right_frontmotor.setIdleMode(IdleMode.kBrake);
    }

    public void disableBrakes()
    {
        left_frontmotor.setIdleMode(IdleMode.kCoast);
        left_backmotor.setIdleMode(IdleMode.kCoast);
        right_backmotor.setIdleMode(IdleMode.kCoast);
        right_frontmotor.setIdleMode(IdleMode.kCoast);
    }

    public void driveBackwards() {

        leftDrive.set(.3);
        rightDrive.set(-.3);

    }

    public void driveForwards() {

        leftDrive.set(-.3);
        rightDrive.set(.3);
    }

    public void correctLeft() {

        leftDrive.set(-.3);
        rightDrive.set(.28);
    }

    public void correctRight() {

        leftDrive.set(-.28);
        rightDrive.set(.3);
    }

    public void driveSetPercentAuto(double Power) {

        leftDrive.set(Power);
        rightDrive.set(-Power);

    }

    public void driveSetPercentForwards(double Power) {

        leftDrive.set(-Power);
        rightDrive.set(Power);

    }

    public void driveStop() {

        leftDrive.set(0);
        rightDrive.set(0);

    }

    // Code added by Panten 3/6/2020

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public double getLeftEncoder() {
        return m_leftEncoder.getPosition();
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getRightEncoder() {
        return m_rightEncoder.getPosition();
    }

}