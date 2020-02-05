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

public class shooterMotorSubsystem extends SubsystemBase {

  CANSparkMax conveyorbeltFrontright = null;
  CANSparkMax conveyorbeltBackright = null;
  CANSparkMax conveyorbeltFrontleft = null;
  CANSparkMax conveyorbeltBackleft = null;
  CANSparkMax shooterleftmotor = null;
  CANSparkMax shooterrightmotor = null;

  public shooterMotorSubsystem() {

    conveyorbeltFrontright = new CANSparkMax(Constants.conveyorbeltFrontrightMotor, null);
    conveyorbeltBackright = new CANSparkMax(Constants.conveyorbeltBackrightMotor, null);
    conveyorbeltFrontleft = new CANSparkMax(Constants.conveyorbeltFrontleftMotor, null);
    conveyorbeltBackleft = new CANSparkMax(Constants.conveyorbeltBackleftMotor, null);
    shooterleftmotor = new CANSparkMax(Constants.shooterleftMotor, null);
    shooterrightmotor = new CANSparkMax(Constants.shooterrightMotor, null);
}

public void shootballsFull() {

  conveyorbeltFrontright.set(1);
  conveyorbeltBackright.set(1);
  conveyorbeltBackleft.set(1);
  conveyorbeltFrontleft.set(1);
  shooterleftmotor.set(.5);
  shooterrightmotor.set(.5);

}

public void shootballsHalf() {

  conveyorbeltFrontright.set(.5);
  conveyorbeltBackright.set(.5);
  conveyorbeltBackleft.set(.5);
  conveyorbeltFrontleft.set(.5);
  shooterleftmotor.set(.5);
  shooterrightmotor.set(.5);

}

public void stopShooter() {

  conveyorbeltFrontright.set(0);
  conveyorbeltBackright.set(0);
  conveyorbeltBackleft.set(0);
  conveyorbeltFrontleft.set(0);
  shooterleftmotor.set(0);
  shooterrightmotor.set(0);
  
}

}