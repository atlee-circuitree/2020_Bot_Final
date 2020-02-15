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


public class levelerMotorSubsystem extends SubsystemBase {

  CANSparkMax levelMotor = null;
   
  public levelerMotorSubsystem() {

  levelMotor = new CANSparkMax(Constants.barmotor, MotorType.kBrushless);


  }
  
  public void levelLeftMotor() {

    levelMotor.set(.5); 
      
  }

  public void levelRightMotor() {

    levelMotor.set(-.5); 
      
  }

  public void stopLevelMotor() {

    levelMotor.set(0); 
      
  }
 
}