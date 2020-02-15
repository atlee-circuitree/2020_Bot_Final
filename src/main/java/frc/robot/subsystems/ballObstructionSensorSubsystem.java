/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ballObstructionSensorSubsystem extends SubsystemBase {
 
DigitalInput input;

  
public ballObstructionSensorSubsystem() {

input = new DigitalInput(0);

}

public boolean isObstructed() {

    return !input.get();
}

  @Override
  public void periodic() {
    
    System.out.println(input.get());

  }
}
