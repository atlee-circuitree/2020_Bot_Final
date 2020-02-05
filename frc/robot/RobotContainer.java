/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.climbdownPnumaticCommand;
import frc.robot.commands.climbupPnumaticCommand;
import frc.robot.commands.closeShooterPnumaticCommand;
import frc.robot.commands.drivetrainCommand;
import frc.robot.commands.intakeSpitballMotorCommand;
import frc.robot.commands.openShooterPnumaticCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.climbPnumaticSubsystem;
import frc.robot.subsystems.drivetrainSubsystem;
import frc.robot.subsystems.kickoutPnumaticSubsystem;
import frc.robot.subsystems.shooterPnumaticSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final drivetrainSubsystem m_drivetrainSubsystem = new drivetrainSubsystem();

  private final climbPnumaticSubsystem m_climbPnumaticSubsystemSubsystem = new climbPnumaticSubsystem();
  private final kickoutPnumaticSubsystem m_kickoutPnumaticSubsystem = new kickoutPnumaticSubsystem();
  private final shooterPnumaticSubsystem m_shooterPnumaticSubsystem = new shooterPnumaticSubsystem();

  //private intakeMotorSubsystem m_intakeMotorSubsystem = new intakeMotorSubsystem();


  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final drivetrainCommand m_drivetrainCommand = new drivetrainCommand();

  private final climbupPnumaticCommand m_climbupPnumaticCommand = new climbupPnumaticCommand();
  private final climbdownPnumaticCommand m_climbdownPnumaticCommand = new climbdownPnumaticCommand();
  private final openShooterPnumaticCommand m_openShooterPnumaticCommand = new openShooterPnumaticCommand();
  private final closeShooterPnumaticCommand m_closeShooterPnumaticCommand = new closeShooterPnumaticCommand();
  private final intakeSpitballMotorCommand m_inIntakeSpitballMotorCommand = new intakeSpitballMotorCommand();

  public static Object driveRobot;

  public static RobotContainer m_RobotContainer;

  public static drivetrainSubsystem drivetrain = new drivetrainSubsystem();

  public static XboxController Xbox1 = new XboxController(0);
  public static XboxController Xbox2 = new XboxController(1);
  public static Joystick Fightstick = new Joystick(2);
  
    JoystickButton O1 = new JoystickButton(Xbox1, 1);
    JoystickButton T1 = new JoystickButton(Xbox2, 1);
    JoystickButton F1 = new JoystickButton(Fightstick, 1);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
    
  }
}

/*  D4.toggleWhenPressed(new climbdownPnumaticCommand());
    D2.toggleWhenPressed(new climbupPnumaticCommand());
    X1.toggleWhenPressed(new openShooterPnumaticCommand());
    X2.toggleWhenPressed(new closeShooterPnumaticCommand());
    D3.toggleWhenPressed(new climbArmupPnumaticCommand());
    D5.toggleWhenPressed(new climbArmdownPnumaticCommand());
    D1.toggleWhenPressed(new kickoutRobotPnumaticCommand());        */