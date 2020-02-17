/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package src.main.java.frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import src.main.java.frc.robot.Constants;


// Code taken from file:///C:/FRC_Code/Tutorials%20and%20other%20junk%20files/FRC%20Programming%20Tutorial%20VSC.pdf

/**
 * Add your docs here.
 */
public class shooterPnumaticSubsystem extends SubsystemBase {
   
  DoubleSolenoid shooterPnumatic = null;
  Compressor compressor = null;

  public shooterPnumaticSubsystem() {

    shooterPnumatic = new DoubleSolenoid(Constants.shooterPnumatic_Deploy, Constants.shooterPnumatic_Retract);
    
  }

  public void openShooter() {

    shooterPnumatic.set(Value.kForward);
    
  }

  public void closeShooter() {

    shooterPnumatic.set(Value.kReverse);
    
  }

}
