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
   
  DoubleSolenoid climbArmPnumatic = null;
  DoubleSolenoid climbHookPnumatic = null;
  public ArmPosition liftArmPosition = ArmPosition.Down;
  public enum ArmPosition
  {
    Up,
    Down
  }

  public climbPnumaticSubsystem() {

    climbArmPnumatic = new DoubleSolenoid(Constants.climbPnumatic_Deploy, Constants.climbPnumatic_Retract);

    climbHookPnumatic = new DoubleSolenoid(Constants.climbArmPnumatic_Deploy, Constants.climbArmPnumatic_Retract);
    
  }

  //Moves the climing arm into the upright position
  public void climbArmUp() {

    climbArmPnumatic.set(Value.kForward);
    liftArmPosition = ArmPosition.Up;

  }

  //moves the climbing arm into the down position
  public void climbArmDown() {

    climbArmPnumatic.set(Value.kReverse);
    liftArmPosition = ArmPosition.Down;
    
  }

  //extends the hook
  public void climbHookExtend() {

    if (liftArmPosition == ArmPosition.Up) {

     climbHookPnumatic.set(Value.kForward);

    } else {

     System.out.println("Error, Climb Arm is not up. Must raise");

    }

  }

  //retracts the hook
  public void climbHookRetract() {

    climbHookPnumatic.set(Value.kForward);
     
  }

}
