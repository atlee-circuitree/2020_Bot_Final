/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;

public class drivetrainSubsystem extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    public drivetrainSubsystem() {

        driveSetup();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void driveSetup() {

        CANSparkMax leftFrontMotor = new CANSparkMax(Constants.driveFrontleftMotor, MotorType.kBrushless);
        CANSparkMax leftBackMotor = new CANSparkMax(Constants.driveBackleftMotor, MotorType.kBrushless);

        CANSparkMax rightFrontMotor = new CANSparkMax(Constants.driveFrontrightMotor, MotorType.kBrushless);
        CANSparkMax rightBackMotor = new CANSparkMax(Constants.driveBackrightMotor, MotorType.kBrushless);

        SpeedControllerGroup leftDrive = new SpeedControllerGroup(leftFrontMotor, leftBackMotor);
        SpeedControllerGroup rightDrive = new SpeedControllerGroup(rightFrontMotor, rightBackMotor);

        DifferentialDrive robotDrive = new DifferentialDrive(leftDrive, rightDrive);

    }

    public void driveRobot() {

        driveSetup();

        roboarcadeDrive(-Xbox1.getY(), Xbox1.getX());
    
  }
}
