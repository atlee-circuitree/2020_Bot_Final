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
  DoubleSolenoid climbHookLeftPnumatic = null;
  DoubleSolenoid climbHookRightPnumatic = null;
  public ClimbArmPosition climbArmPosition = ClimbArmPosition.DOWN;
  public ClimbHookPosition climbHookPosition = ClimbHookPosition.RETRACTED;
  public enum ClimbArmPosition
  {
    UP,
    DOWN
  }
  public enum ClimbHookPosition
  {
    EXTENDED,
    RETRACTED
  }

  public climbPnumaticSubsystem() {

    climbArmPnumatic = new DoubleSolenoid(Constants.climbArmPnumatic_Deploy, Constants.climbArmPnumatic_Retract);

    climbHookLeftPnumatic = new DoubleSolenoid(Constants.climbHookLeftPnumatic_Deploy, Constants.climbHookLeftPnumatic_Retract);

    climbHookRightPnumatic = new DoubleSolenoid(Constants.climbHookRightPnumatic_Deploy, Constants.climbHookRightPnumatic_Retract);
    
  }

  //Moves the climing arm into the upright position
  public void climbArmUp() {

    climbArmPnumatic.set(Value.kForward);
    climbArmPosition = ClimbArmPosition.UP;

  }

  //moves the climbing arm into the down position
  public void climbArmDown() {

    if (climbHookPosition == ClimbHookPosition.RETRACTED) {

    climbArmPnumatic.set(Value.kReverse);

    } else {

      System.out.println("Error, Climb Hook is extended. Must retract");

    }

    climbArmPosition = ClimbArmPosition.DOWN;
    
  }

  //extends the hook
  public void climbHookExtend() {

    if (climbArmPosition == ClimbArmPosition.UP) {

     climbHookLeftPnumatic.set(Value.kForward);

    } else {

      System.out.println("Error, Climb Arm is not up. Must raise");

    }

    climbHookPosition = ClimbHookPosition.EXTENDED;

  }

  //retracts the hook
  public void climbHookRetract() {

    climbHookLeftPnumatic.set(Value.kReverse);
     
    climbHookPosition = ClimbHookPosition.RETRACTED;

  }

  
}
