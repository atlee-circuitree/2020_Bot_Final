/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.climbdownPnumaticCommand;
import frc.robot.commands.climbupPnumaticCommand;
import frc.robot.commands.closeShooterPnumaticCommand;
import frc.robot.commands.drivetrainCommand;
import frc.robot.commands.intakeSpitballMotorCommand;
import frc.robot.commands.intakeTakeballMotorCommand;
import frc.robot.commands.kickoutPnumaticCommand;
import frc.robot.commands.openShooterPnumaticCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.climbPnumaticSubsystem;
import frc.robot.subsystems.drivetrainSubsystem;
import frc.robot.subsystems.intakeMotorSubsystem;
import frc.robot.subsystems.kickoutPnumaticSubsystem;
import frc.robot.subsystems.shooterPnumaticSubsystem;
import frc.robot.Constants;

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


 private static drivetrainSubsystem m_drivetrainSubsystem = new drivetrainSubsystem();
 private static drivetrainCommand c_dDrivetrainCommand;

 /*
  private final climbPnumaticSubsystem m_climbPnumaticSubsystem = new climbPnumaticSubsystem();
  private final kickoutPnumaticSubsystem m_kickoutPnumaticSubsystem = new kickoutPnumaticSubsystem();
  private final shooterPnumaticSubsystem m_shooterPnumaticSubsystem = new shooterPnumaticSubsystem();

  private final intakeMotorSubsystem m_intakeMotorSubsystem = new intakeMotorSubsystem();
*/
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
/*
  private final climbupPnumaticCommand m_ClimbupPnumaticCommand = new climbupPnumaticCommand();
  private final climbdownPnumaticCommand m_climbdownPnumaticCommand = new climbdownPnumaticCommand();
  private final openShooterPnumaticCommand m_openShooterPnumaticCommand = new openShooterPnumaticCommand();
  private final closeShooterPnumaticCommand m_closeShooterPnumaticCommand = new closeShooterPnumaticCommand();
  private final kickoutPnumaticCommand m_kickoutPnumaticCommand = new kickoutPnumaticCommand();

  private final intakeSpitballMotorCommand m_inIntakeSpitballMotorCommand = new intakeSpitballMotorCommand();
  private final intakeTakeballMotorCommand m_inIntakeTakeballMotorCommand = new intakeTakeballMotorCommand();
  */ 

  public static Object driveRobot;

  public static RobotContainer m_RobotContainer;

  public static XboxController Xbox1 = new XboxController(0);
  public static XboxController Xbox2 = new XboxController(1);
  public static Joystick Fightstick = new Joystick(2);
  

    JoystickButton O1 = new JoystickButton(Xbox1, 1); //Take Balls
    JoystickButton O2 = new JoystickButton(Xbox1, 2); //Spit Balls
    JoystickButton O3 = new JoystickButton(Xbox1, 3); 
    JoystickButton O4 = new JoystickButton(Xbox1, 4);
    JoystickButton O5 = new JoystickButton(Xbox1, 5); 
    JoystickButton O6 = new JoystickButton(Xbox1, 6);
    JoystickButton O7 = new JoystickButton(Xbox1, 7);   
    JoystickButton O8 = new JoystickButton(Xbox1, 8);

    JoystickButton T1 = new JoystickButton(Xbox2, 1); //Shoot Full Speed
    JoystickButton T2 = new JoystickButton(Xbox2, 2); //Shoot Half Speed
    JoystickButton T3 = new JoystickButton(Xbox2, 3); //Open Shooter
    JoystickButton T4 = new JoystickButton(Xbox2, 4); //Close Shooter
    JoystickButton T5 = new JoystickButton(Xbox2, 5);
    JoystickButton T6 = new JoystickButton(Xbox2, 6);
    JoystickButton T7 = new JoystickButton(Xbox2, 7);
    JoystickButton T8 = new JoystickButton(Xbox2, 8);

    JoystickButton F1 = new JoystickButton(Fightstick, 1); //Kickout Robot
    JoystickButton F2 = new JoystickButton(Fightstick, 2); //Climb Up
    JoystickButton F3 = new JoystickButton(Fightstick, 3); //Climb Arm Up
    JoystickButton F4 = new JoystickButton(Fightstick, 4); //Climb Down
    JoystickButton F5 = new JoystickButton(Fightstick, 5); //Climb Arm Down
    JoystickButton F6 = new JoystickButton(Fightstick, 6);
    JoystickButton F7 = new JoystickButton(Fightstick, 7);
    JoystickButton F8 = new JoystickButton(Fightstick, 8);

    
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    setUpDrive();

    configureButtonBindings();


  }

  //Operations specific to TeleOp only
  public void teleopInit()
  {
    c_dDrivetrainCommand = new drivetrainCommand(Xbox1, m_drivetrainSubsystem);
    m_drivetrainSubsystem.setDefaultCommand(c_dDrivetrainCommand);
  }

  public void setUpDrive() {

    CANSparkMax leftFrontMotor = new CANSparkMax(Constants.driveFrontleftMotor, MotorType.kBrushless);
    CANSparkMax leftBackMotor = new CANSparkMax(Constants.driveBackleftMotor, MotorType.kBrushless);

    CANSparkMax rightFrontMotor = new CANSparkMax(Constants.driveFrontrightMotor, MotorType.kBrushless);
    CANSparkMax rightBackMotor = new CANSparkMax(Constants.driveBackrightMotor, MotorType.kBrushless);

    m_drivetrainSubsystem.driveSetup(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

   /* 
    O1.toggleWhenPressed(m_inIntakeTakeballMotorCommand);
    O2.toggleWhenPressed(m_inIntakeSpitballMotorCommand);
    O3.toggleWhenPressed(m_autoCommand);
    O4.toggleWhenPressed(m_autoCommand);
    O5.toggleWhenPressed(m_autoCommand);
    O6.toggleWhenPressed(m_autoCommand);
    O7.toggleWhenPressed(m_autoCommand);
    O8.toggleWhenPressed(m_autoCommand);

    T1.toggleWhenPressed(m_openShooterPnumaticCommand);
    T2.toggleWhenPressed(m_closeShooterPnumaticCommand);
    T3.toggleWhenPressed(m_autoCommand);
    T4.toggleWhenPressed(m_autoCommand);
    T5.toggleWhenPressed(m_autoCommand);
    T6.toggleWhenPressed(m_autoCommand);
    T7.toggleWhenPressed(m_autoCommand);
    T8.toggleWhenPressed(m_autoCommand);

    F1.toggleWhenPressed(m_kickoutPnumaticCommand);
    F2.toggleWhenPressed(m_autoCommand);
    F3.toggleWhenPressed(m_autoCommand);
    F4.toggleWhenPressed(m_autoCommand);
    F5.toggleWhenPressed(m_autoCommand);
    F6.toggleWhenPressed(m_autoCommand);
    F7.toggleWhenPressed(m_autoCommand);
    F8.toggleWhenPressed(m_autoCommand);
    */


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