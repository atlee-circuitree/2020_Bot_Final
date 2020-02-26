/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class wheelMotorSubsystem extends SubsystemBase {

  CANSparkMax wheelMotor = null;
   
  public wheelMotorSubsystem() {

  wheelMotor = new CANSparkMax(Constants.wheelMotor, MotorType.kBrushless);


  }
  
  public void spinWheelMotor() {

    wheelMotor.set(.65); 
      
  }

  public void stopWheelMotor() {

    wheelMotor.set(0); 
      
  }
 
 
}