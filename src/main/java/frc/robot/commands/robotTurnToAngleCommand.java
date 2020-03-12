/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.DrivetrainSubsystem;

public class robotTurnToAngleCommand extends CommandBase {

    DrivetrainSubsystem m_driveSubsystem;
    double idealAngle;
    PIDController turnController = new PIDController(0.03, 0,0);

    
    public robotTurnToAngleCommand(DrivetrainSubsystem drivetrainSubsystem, double targetAngle) {

        super();
        turnController.setTolerance(1);
        m_driveSubsystem = drivetrainSubsystem;
        idealAngle = targetAngle;
        addRequirements(m_driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        turnController.setSetpoint(m_driveSubsystem.getContinuousAngle() + idealAngle); //read the current angle, add on ideal angle
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double turnValue = MathUtil.clamp(turnController.calculate(m_driveSubsystem.getContinuousAngle()), -0.5, 0.5);
        m_driveSubsystem.driveRobotLinear(0d, turnValue);
    }

    @Override
    public boolean isFinished() {
        return turnController.atSetpoint();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.driveStop();
    }

}
