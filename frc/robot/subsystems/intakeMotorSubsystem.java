/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class intakeMotorSubsystem extends SubsystemBase {

  CANSparkMax intake = null;

  public intakeMotorSubsystem() {

  intake = new CANSparkMax(Constants.intakeMotor, null);
}

public void takeinballs() {

  intake.set(1);

}

public void spitoutballs() {

  intake.set(-1);
  
}

public void stopintake() {

  intake.set(0);
    
}

}