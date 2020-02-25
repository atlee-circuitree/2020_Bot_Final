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
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

public class shooterMotorSubsystem extends SubsystemBase {

  TalonSRX rightShooter = null;
  TalonSRX leftShooter = null;
  
   
  public shooterMotorSubsystem() {

  rightShooter = new TalonSRX(Constants.rightShooter);
  leftShooter = new TalonSRX(Constants.leftShooter);

  
}

public void runShooter() {

  //rightShooter.set(ControlMode.Velocity, -900);
  //leftShooter.set(ControlMode.Velocity, 900);
  rightShooter.set(ControlMode.PercentOutput, -.75);
  leftShooter.set(ControlMode.PercentOutput, .75);
}

public void runShooterEncoder() {

   rightShooter.set(ControlMode.Position, 100000);
   leftShooter.set(ControlMode.Position, 100000);

}

public void runShooter50() {

  rightShooter.set(ControlMode.PercentOutput, -.48);
  leftShooter.set(ControlMode.PercentOutput, .48);
}

public void stopShooter(){
   
  rightShooter.set(TalonSRXControlMode.Velocity, 0);
  leftShooter.set(TalonSRXControlMode.Velocity, 0);
}


public void shooterOnly() {

  //conveyorbeltRight.set(0);
  //conveyorbeltLeft.set(0);
 // rightShooter.set(TalonSRXControlMode.Velocity, -1000);
 // leftShooter.set(TalonSRXControlMode.Velocity, 1000);
  rightShooter.set(ControlMode.PercentOutput, -.75);
  leftShooter.set(ControlMode.PercentOutput, .75);


}
 


}