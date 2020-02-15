/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.Compressor;

// Code taken from file:///C:/FRC_Code/Tutorials%20and%20other%20junk%20files/FRC%20Programming%20Tutorial%20VSC.pdf

/**
 * Add your docs here.
 */
public class climbPnumaticSubsystem extends SubsystemBase {
   
  DoubleSolenoid leftClimbPnumatic = null;
  DoubleSolenoid rightClimbPnumatic = null;
  DoubleSolenoid leftClimbArmPnumatic = null;
  DoubleSolenoid rightClimbArmPnumatic = null;
  

  public void climber() {

    leftClimbPnumatic = new DoubleSolenoid(Constants.climbPnumatic_Deploy, Constants.climbPnumatic_Retract);
    rightClimbPnumatic = new DoubleSolenoid(Constants.climbPnumatic_Deploy, Constants.climbPnumatic_Retract);
    leftClimbArmPnumatic = new DoubleSolenoid(Constants.climbArmPnumatic_Deploy, Constants.climbArmPnumatic_Retract);
    rightClimbArmPnumatic = new DoubleSolenoid(Constants.climbArmPnumatic_Deploy, Constants.climbArmPnumatic_Retract);
    
  }

  public void climbUp() {

    leftClimbPnumatic.set(Value.kForward);
    rightClimbPnumatic.set(Value.kForward);

  }

  public void climbDown() {

    leftClimbPnumatic.set(Value.kReverse);
    rightClimbPnumatic.set(Value.kReverse);

  }

  public void climbArmUp() {

    leftClimbArmPnumatic.set(Value.kForward);
    rightClimbArmPnumatic.set(Value.kForward);

  }

  public void climbArmDown() {

    leftClimbArmPnumatic.set(Value.kReverse);
    rightClimbArmPnumatic.set(Value.kReverse);

  }

}
