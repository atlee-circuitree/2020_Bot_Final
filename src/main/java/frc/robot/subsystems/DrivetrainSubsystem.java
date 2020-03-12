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
import com.kauailabs.navx.frc.AHRS;

public class DrivetrainSubsystem extends SubsystemBase {

    SpeedControllerGroup leftDrive;
    SpeedControllerGroup rightDrive;
    DifferentialDrive robotDrive;
    AHRS ahrs;

    CANSparkMax left_frontmotor;
    CANSparkMax left_backmotor;
    CANSparkMax right_frontmotor;
    CANSparkMax right_backmotor;


    PIDController turnController;

    /**
     * Creates a new ExampleSubsystem.
     */
    
    public DrivetrainSubsystem() {

        ahrs = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Added 3/6/2020 Panten
        // Update the odometry in the periodic block
        if (m_rightEncoder != null) // make sure setup has been called first
        {
            double heading = getHeading();
            double leftPos = m_leftEncoder.getPosition();
            double rightPos = m_rightEncoder.getPosition();

            m_odometry.update(Rotation2d.fromDegrees(heading), leftPos,
                rightPos);
            
            SmartDashboard.putNumber("Heading", heading);
            SmartDashboard.putNumber("LeftPos", leftPos);
            SmartDashboard.putNumber("RightPos", rightPos);

        }
    }

    public void driveSetup(CANSparkMax leftFrontMotor, CANSparkMax leftBackMotor, CANSparkMax rightFrontMotor,
            CANSparkMax rightBackMotor) {
        m_leftEncoder = leftFrontMotor.getEncoder();
        m_rightEncoder = rightFrontMotor.getEncoder();

        left_frontmotor = leftFrontMotor;
        left_backmotor = leftBackMotor;
        right_frontmotor = rightFrontMotor;
        right_backmotor = rightBackMotor;

        leftDrive = new SpeedControllerGroup(leftFrontMotor, leftBackMotor);
        rightDrive = new SpeedControllerGroup(rightFrontMotor, rightBackMotor);
        m_drive = new DifferentialDrive(leftDrive, rightDrive);
        robotDrive = new DifferentialDrive(leftDrive, rightDrive);

        // code added for pathfinder by Panten 3/6/2020
        // Sets the distance per pulse for the encoders
        // set scaling factor for CANEncoder.getPosition() so that it matches the output
        // of
        // Encoder.getDistance() method.

        //42 encoder ticks per motor shaft revolution - removed 3/11 by Berg - getPosition is motor revolutions - encoder doesn't factor into the calculation
        //Gear box reduction of 10.71
        //Wheels 8 inch diameter
        //Converted to meters
        //Result = pulses per meter.  Invert to make meters per pulse
        double conversionFactor = 1/(((10.71/(8*Math.PI)))/0.0254);  
        m_leftEncoder
                .setPositionConversionFactor(conversionFactor);
        m_rightEncoder
                .setPositionConversionFactor(conversionFactor);

        // Native scale is RPM. Scale velocity so that it is in meters/sec
        m_leftEncoder.setVelocityConversionFactor(
            conversionFactor / 60.0);
        m_rightEncoder.setVelocityConversionFactor(
            conversionFactor / 60.0);
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
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

    public void printPose() {

        System.out.println(getPose());

    }

    // Code added by Panten 3/6/2020

    // The robot's drive
    DifferentialDrive m_drive;

    // The left-side drive encoder
    CANEncoder m_leftEncoder;

    // The right-side drive encoder
    CANEncoder m_rightEncoder;

    // Odometry class for tracking robot pose
    DifferentialDriveOdometry m_odometry;

    /**
     * Creates a new DriveSubsystem.
     */
    /*
     * Moved to DrivetrainSubsystem by Panten 3/6/2020 public DriveSubsystem() { //
     * Sets the distance per pulse for the encoders
     * m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
     * m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
     * 
     * resetEncoders(); m_odometry = new
     * DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading())); }
     * 
     * 
     * @Override //Moved to periodic public void periodic() { // Update the odometry
     * in the periodic block m_odometry.update(Rotation2d.fromDegrees(getHeading()),
     * m_leftEncoder.getDistance(), m_rightEncoder.getDistance()); }
     */

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        //ahrs.zeroYaw();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftDrive.setVoltage(leftVolts);
        rightDrive.setVoltage(-rightVolts);
        m_drive.feed();
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

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
    public CANEncoder getLeftEncoder() {
        return m_leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public CANEncoder getRightEncoder() {
        return m_rightEncoder;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        ahrs.zeroYaw();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        // return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.kGyroReversed
        // ? -1.0 : 1.0);
        return Math.IEEEremainder(ahrs.getYaw(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
        //return Math.IEEEremainder(ahrs.getFusedHeading() - 180, 360) * (Constants.kGyroReversed ? -1.0 : 1.0); //potential test in case Yaw isn't accurate enough
    }

    public double getContinuousAngle()
    {
        return ahrs.getAngle();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return ahrs.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
    }

}