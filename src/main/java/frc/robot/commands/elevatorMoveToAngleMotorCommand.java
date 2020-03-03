/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IMUSubsystem;
import frc.robot.subsystems.elevatorMotorSubsystem;

public class elevatorMoveToAngleMotorCommand extends CommandBase {

    elevatorMotorSubsystem m_elevatorMotorSubsystem;
    IMUSubsystem m_IMUSubsystem;
    double idealAngle;

    public elevatorMoveToAngleMotorCommand(elevatorMotorSubsystem motorSubsystem, IMUSubsystem sensorSubsystem, double targetAngle) {

        super();
        idealAngle = targetAngle;
        m_elevatorMotorSubsystem = motorSubsystem;
        m_IMUSubsystem = sensorSubsystem;
        addRequirements(m_elevatorMotorSubsystem);
        addRequirements(m_IMUSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (m_IMUSubsystem.getTilt() > idealAngle) {
            m_elevatorMotorSubsystem.moveShooterDown();

        } else 
        {
            m_elevatorMotorSubsystem.moveShooterUp();
        }
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_IMUSubsystem.getTilt() - idealAngle) < 0.05) {
            return true;
        }
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_elevatorMotorSubsystem.stopElevator();
    }

}
