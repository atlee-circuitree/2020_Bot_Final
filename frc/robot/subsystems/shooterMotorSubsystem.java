/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class shooterMotorSubsystem extends SubsystemBase {

  CANSparkMax intake = null;
  CANSparkMax conveyorbeltLeft = null;
  CANSparkMax conveyorbeltRight = null;
   
  public shooterMotorSubsystem() {

  intake = new CANSparkMax(Constants.intakeMotor, MotorType.kBrushless);
  conveyorbeltLeft = new CANSparkMax(Constants.conveyorbeltRight, MotorType.kBrushless);
  conveyorbeltRight = new CANSparkMax(Constants.conveyorbeltLeft, MotorType.kBrushless);

}

public void takeinballs() {

  intake.set(1);
  conveyorbeltRight.set(-.5);
  conveyorbeltLeft.set(.5);
  
}

public void spitoutballs() {

  intake.set(-1);
  conveyorbeltRight.set(.5);
  conveyorbeltLeft.set(-.5);
  
}

public void stopintake() {

  intake.set(0); 
  conveyorbeltRight.set(0);
  conveyorbeltLeft.set(0);
    
}
}