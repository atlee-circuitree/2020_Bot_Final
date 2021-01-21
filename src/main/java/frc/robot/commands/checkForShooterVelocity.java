/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ballObstructionSensorSubsystem;
import frc.robot.subsystems.shooterIntakeSubsystem;
import frc.robot.subsystems.shooterMotorSubsystem;

public class checkForShooterVelocity extends CommandBase {

    shooterMotorSubsystem m_subsystem;

    public checkForShooterVelocity(shooterMotorSubsystem shooterSubsystem) {

        super();

        m_subsystem = shooterSubsystem;

        addRequirements(m_subsystem);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double velocity = m_subsystem.getVelocityError();

        if ((Math.abs(velocity) < 300)) {

            return true;

        }
        return false;
    }
}
