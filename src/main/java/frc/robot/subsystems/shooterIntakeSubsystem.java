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

public class shooterIntakeSubsystem extends SubsystemBase {

  CANSparkMax intake = null;
  CANSparkMax conveyorbeltLeft = null;
  CANSparkMax conveyorbeltRight = null;
  CANPIDController m_shooterleftPIDController;
  CANPIDController m_shooterrightPIDController;

   
  public shooterIntakeSubsystem() {

  intake = new CANSparkMax(Constants.intakeMotor, MotorType.kBrushless);
  conveyorbeltLeft = new CANSparkMax(Constants.conveyorbeltRight, MotorType.kBrushless);
  conveyorbeltRight = new CANSparkMax(Constants.conveyorbeltLeft, MotorType.kBrushless);

  m_shooterleftPIDController = intake.getPIDController();

  //42 ticks

  intake.getEncoder();

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

public void straightenballs() {

  conveyorbeltRight.set(-1);
  conveyorbeltLeft.set(1);

}

public void stopConveyor(){

  conveyorbeltRight.set(0);
  conveyorbeltLeft.set(0);

}

public void conveyorOnly(){

  conveyorbeltRight.set(-.7);
  conveyorbeltLeft.set(.7);
  
}

public void encoderIntake() {

   

}

public void waitOneSecond() {



}

}