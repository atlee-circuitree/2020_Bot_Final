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


public class shooterMotorSubsystem extends SubsystemBase {

  CANSparkMax intake = null;
  CANSparkMax conveyorbeltLeft = null;
  CANSparkMax conveyorbeltRight = null;
  TalonSRX rightShooter = null;
  TalonSRX leftShooter = null;


   
  public shooterMotorSubsystem() {

  intake = new CANSparkMax(Constants.intakeMotor, MotorType.kBrushless);
  conveyorbeltLeft = new CANSparkMax(Constants.conveyorbeltRight, MotorType.kBrushless);
  conveyorbeltRight = new CANSparkMax(Constants.conveyorbeltLeft, MotorType.kBrushless);
  rightShooter = new TalonSRX(Constants.rightShooter);
  leftShooter = new TalonSRX(Constants.leftShooter);

}

public void takeinballs() {

  intake.set(.7);
  conveyorbeltRight.set(-.5);
  conveyorbeltLeft.set(.5);
  
}

public void spitoutballs() {

  intake.set(-.7);
  conveyorbeltRight.set(0);
  conveyorbeltLeft.set(0);
  
}

public void stopintake() {

  intake.set(0); 
  conveyorbeltRight.set(0);
  conveyorbeltLeft.set(0);
    
}
public void runShooter() {

  //conveyorbeltRight.set(-.7);
  //conveyorbeltLeft.set(.7);
  rightShooter.set(ControlMode.PercentOutput, -.7);
  leftShooter.set(ControlMode.PercentOutput, .7);
}
public void stopShooter(){
  conveyorbeltRight.set(0);
  conveyorbeltLeft.set(0);
  rightShooter.set(TalonSRXControlMode.Velocity, 0);
  leftShooter.set(TalonSRXControlMode.Velocity, 0);
}

public void conveyorOnly(){
  conveyorbeltRight.set(.5);
  conveyorbeltLeft.set(.5);
  rightShooter.set(TalonSRXControlMode.Velocity, 0);
  leftShooter.set(TalonSRXControlMode.Velocity, 0);
}

public void shooterOnly() {

  conveyorbeltRight.set(0);
  conveyorbeltLeft.set(0);
  rightShooter.set(TalonSRXControlMode.Velocity, -.8);
  leftShooter.set(TalonSRXControlMode.Velocity, .8);

}

}