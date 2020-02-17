/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package src.main.java.frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import src.main.java.frc.robot.Constants;


public class elevatorMotorSubsystem extends SubsystemBase {

  CANSparkMax elevatorMotor = null;

  public elevatorMotorSubsystem() {

  elevatorMotor = new CANSparkMax(Constants.elevatorMotor, MotorType.kBrushless);

}

public void moveShooterUp() {

  elevatorMotor.set(.25);

}

public void moveShooterDown() {

  elevatorMotor.set(-.25);
   
}

public void stopElevator() {

  elevatorMotor.set(0); 
    
}
}