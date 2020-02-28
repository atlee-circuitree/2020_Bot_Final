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
  
  public int shooterVelocity = (leftShooter.getSelectedSensorVelocity(1));

  StringBuilder _sb = new StringBuilder();

  public ShooterMotorStatus shooterMotorStatus = ShooterMotorStatus.IS_NOT_RUNNING;

  public enum ShooterMotorStatus
  {
    IS_RUNNING,
    IS_NOT_RUNNING
  }

  public shooterMotorSubsystem() {

  rightShooter = new TalonSRX(Constants.rightShooter);
  leftShooter = new TalonSRX(Constants.leftShooter);
  rightShooter.follow(leftShooter);
  rightShooter.setInverted(true);
  leftShooter.setInverted(false);
  
}

public void runShooter() {
 
  if (shooterMotorStatus == ShooterMotorStatus.IS_RUNNING) {
    
    leftShooter.set(ControlMode.Velocity, 5485);
    //15500 75% 14000
    //9300 15000
    //7905 13000
    //6500 10000
    //5750 9500 50% Inner
    //5500 9100 66% Inner
    //5485
    //5475 9046 80% Inner
    //5400 8950
    //5000 8346

  } else {

    //rightShooter.set(ControlMode.PercentOutput, 0);
    leftShooter.set(ControlMode.PercentOutput, 0);

  }

}

public void flipShooterState() {

  if (shooterMotorStatus == ShooterMotorStatus.IS_RUNNING) {

    shooterMotorStatus = ShooterMotorStatus.IS_NOT_RUNNING;

  } else {

    shooterMotorStatus = ShooterMotorStatus.IS_RUNNING;

  }

}

public void runShooterEncoder() {

   //rightShooter.set(ControlMode.Position, 100000);
   leftShooter.set(ControlMode.Position, 100000);

}

public int getVelocity() {
  System.out.print(" Velocity ");
  System.out.print(leftShooter.getSelectedSensorVelocity(1));
  System.out.print(" Input ");
  System.out.print(leftShooter.getClosedLoopError());
  System.out.print(" ErrorD ");
  System.out.println(leftShooter.getErrorDerivative());

  return leftShooter.getClosedLoopError(0);
}

public void runShooter50() {

  //rightShooter.set(ControlMode.PercentOutput, -.48);
  leftShooter.set(ControlMode.PercentOutput, .48);
}

public void stopShooter(){
   
  //rightShooter.set(TalonSRXControlMode.PercentOutput, 0);
  leftShooter.set(TalonSRXControlMode.PercentOutput, 0);
}


public void shooterOnly() {

  //rightShooter.set(ControlMode.PercentOutput, -.75);
  leftShooter.set(ControlMode.PercentOutput, .75);


}
 
@Override
public void periodic() {
  //getVelocity();  
}

}