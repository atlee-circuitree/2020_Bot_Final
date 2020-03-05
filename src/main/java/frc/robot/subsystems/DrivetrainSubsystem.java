/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
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
    /**
     * Creates a new ExampleSubsystem.
     */
    public DrivetrainSubsystem() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void driveSetup(CANSparkMax leftFrontMotor, CANSparkMax leftBackMotor, CANSparkMax rightFrontMotor, CANSparkMax rightBackMotor) {
        
        
        leftDrive = new SpeedControllerGroup(leftFrontMotor, leftBackMotor);
        rightDrive = new SpeedControllerGroup(rightFrontMotor, rightBackMotor);

        robotDrive = new DifferentialDrive(leftDrive, rightDrive);

        //robotDrive.arcadeDrive(-Xbox1.getY(), Xbox1.getX());

    }

    public void driveRobot(Double X, double Y) {

        robotDrive.arcadeDrive(-Y, X, true);
    }

    public void driveBackwards() {

    leftDrive.set(.3);
    rightDrive.set(-.3);

    }

    public void driveSetPercent(double Power) {

        leftDrive.set(Power);
        rightDrive.set(-Power);
    
        }

    public void driveStop() {

        leftDrive.set(0);
        rightDrive.set(0);
    
    }

}