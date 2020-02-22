/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD licenseaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// FROM LARSON!! https://www.chiefdelphi.com/t/error-message-from-robotbase/162791/12

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TimerCommand;
import frc.robot.commands.autonomusCommand2020;
import frc.robot.commands.ballObstructionSensorCommand;
import frc.robot.commands.climbarmdownPnumaticCommand;
import frc.robot.commands.climbarmupPnumaticCommand;
import frc.robot.commands.climbdownPnumaticCommand;
import frc.robot.commands.climbupPnumaticCommand;
import frc.robot.commands.closeShooterPnumaticCommand;
import frc.robot.commands.conveyorbeltObstructedCommand;
import frc.robot.commands.conveyorbeltclearCommand;
import frc.robot.commands.drivetrainCommand;
import frc.robot.commands.drivetrainPercentPowerAuto;
import frc.robot.commands.elevatorMotorCommand;
import frc.robot.commands.intakeSpitballMotorCommand;
import frc.robot.commands.intakeTakeballMotorCommand;
import frc.robot.commands.kickoutPnumaticCommand;
import frc.robot.commands.kickoutReversePnumaticCommand;
import frc.robot.commands.levelerLeftMotorCommand;
import frc.robot.commands.levelerRightMotorCommand;
import frc.robot.commands.openShooterPnumaticCommand;
import frc.robot.commands.runShooter50MotorCommand;
import frc.robot.commands.runShooterMotorCommand;
import frc.robot.commands.shooterOnlyConveyorMotorCommand;
import frc.robot.commands.shooterOnlyMotorCommand;
import frc.robot.commands.spinWheelMotorCommand;
import frc.robot.commands.stopConveyorMotorCommand;
import frc.robot.commands.stopShooterMotorCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ballObstructionSensorSubsystem;
import frc.robot.subsystems.drivetrainSubsystem;
import frc.robot.subsystems.elevatorMotorSubsystem;
import frc.robot.subsystems.kickoutPnumaticSubsystem;
import frc.robot.subsystems.levelerMotorSubsystem;
import frc.robot.subsystems.shooterMotorSubsystem;
import frc.robot.subsystems.shooterIntakeSubsystem;
import frc.robot.subsystems.shooterPnumaticSubsystem;
import frc.robot.subsystems.wheelMotorSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.climbPnumaticSubsystem;
import frc.robot.subsystems.ballObstructionSensorSubsystem;
import frc.robot.commands.ballObstructionSensorCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
  private static elevatorMotorCommand c_delevatorMotorCommand;

  // private final climbPnumaticSubsystem m_climbPnumaticSubsystem = new
  // climbPnumaticSubsystem();
  // private final kickoutPnumaticSubsystem m_kickoutPnumaticSubsystem = new
  // kickoutPnumaticSubsystem();
  private final shooterPnumaticSubsystem m_shooterPnumaticSubsystem = new shooterPnumaticSubsystem();

  private final autonomusCommand2020 m_autonomusCommand2020 = new autonomusCommand2020();

  private final shooterMotorSubsystem m_shooterMotorSubsystem = new shooterMotorSubsystem();
  private final shooterIntakeSubsystem m_shooterIntakeSubsystem = new shooterIntakeSubsystem();

  private final climbPnumaticSubsystem m_climbPnumaticSubsystem = new climbPnumaticSubsystem();

  private final elevatorMotorSubsystem m_elevatorMotorSubsystem = new elevatorMotorSubsystem();

  private final levelerMotorSubsystem m_levelerMotorSubsystem = new levelerMotorSubsystem();

  //private final autonomusCommand2020 m_autonomusCommand2020 = new autonomusCommand2020();

  private final WaitCommand m_Wait500Command = new WaitCommand(.5);

  private final WaitCommand m_Wait1000Command = new WaitCommand(1);

  private final WaitCommand m_Wait2000Command = new WaitCommand(2);

  private final kickoutPnumaticSubsystem m_kickoutPnumaticSubsystem = new kickoutPnumaticSubsystem();
  private final ballObstructionSensorSubsystem m_bBallObstructionSensorSubsystem = new ballObstructionSensorSubsystem();
  private final wheelMotorSubsystem m_wheelMotorSubsystem = new wheelMotorSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final climbupPnumaticCommand m_climbupPnumaticCommand = new climbupPnumaticCommand(m_climbPnumaticSubsystem);
  private final climbdownPnumaticCommand m_climbdownPnumaticCommand = new climbdownPnumaticCommand(
      m_climbPnumaticSubsystem);
  private final climbarmupPnumaticCommand m_climbarmupPnumaticCommand = new climbarmupPnumaticCommand(
      m_climbPnumaticSubsystem);
  private final climbarmdownPnumaticCommand m_climbarmdownPnumaticCommand = new climbarmdownPnumaticCommand(
      m_climbPnumaticSubsystem);
  private final openShooterPnumaticCommand m_openShooterPnumaticCommand = new openShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);
  private final openShooterPnumaticCommand m_openShooterPnumaticCommand2 = new openShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);
  private final openShooterPnumaticCommand m_openShooterPnumaticCommand3 = new openShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);
  private final closeShooterPnumaticCommand m_closeShooterPnumaticCommand = new closeShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);
  private final closeShooterPnumaticCommand m_closeShooterPnumaticCommand2 = new closeShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);


  private final runShooterMotorCommand m_runShooterMotorCommand = new runShooterMotorCommand(m_shooterMotorSubsystem);

  private final runShooterMotorCommand m_runShooterMotorCommand2 = new runShooterMotorCommand(m_shooterMotorSubsystem);

  private final stopShooterMotorCommand m_stopShooterMotorCommand = new stopShooterMotorCommand(m_shooterMotorSubsystem);

  private final stopShooterMotorCommand m_stopShooterMotorCommand2 = new stopShooterMotorCommand(m_shooterMotorSubsystem);

  private final stopConveyorMotorCommand m_stopConveyorMotorCommand = new stopConveyorMotorCommand(m_shooterIntakeSubsystem);

  private final shooterOnlyConveyorMotorCommand m_shooterOnlyConveyorMotorCommand2 = new shooterOnlyConveyorMotorCommand(m_shooterIntakeSubsystem);

  private final shooterOnlyConveyorMotorCommand m_shooterOnlyConveyorMotorCommand = new shooterOnlyConveyorMotorCommand(m_shooterIntakeSubsystem);

  private final shooterOnlyMotorCommand m_shooterOnlyMotorCommand = new shooterOnlyMotorCommand(m_shooterMotorSubsystem);
  
  private final kickoutPnumaticCommand m_kickoutPnumaticCommand = new kickoutPnumaticCommand(m_kickoutPnumaticSubsystem);

  private final spinWheelMotorCommand m_spinWheelMotorCommand = new spinWheelMotorCommand(m_wheelMotorSubsystem);

  private final levelerLeftMotorCommand m_levelerLeftMotorCommand = new levelerLeftMotorCommand(m_levelerMotorSubsystem);

  private final levelerRightMotorCommand m_levelerRightMotorCommand = new levelerRightMotorCommand(m_levelerMotorSubsystem);

  private final ballObstructionSensorCommand m_BallObstructionSensorCommand = new ballObstructionSensorCommand(m_bBallObstructionSensorSubsystem);

  private final kickoutReversePnumaticCommand m_kickoutReversePnumaticCommand = new kickoutReversePnumaticCommand(m_kickoutPnumaticSubsystem);

  private final intakeTakeballMotorCommand m_intakeTakeballMotorCommand = new intakeTakeballMotorCommand(m_shooterIntakeSubsystem, m_bBallObstructionSensorSubsystem);

  private final intakeSpitballMotorCommand m_intakeSpitballMotorCommand = new intakeSpitballMotorCommand(m_shooterIntakeSubsystem);

  private final runShooter50MotorCommand m_runShooter50MotorCommand = new runShooter50MotorCommand(m_shooterMotorSubsystem, false);

  private final TimerCommand m_centerDriveBackCommand = new TimerCommand(1000);
  
  private final TimerCommand m_shooterWarmupTimerCommand = new TimerCommand(1000);

  //This is for autonomous to clear all three balls
  private final TimerCommand m_shooterConveyorTimerCommand = new TimerCommand(2000);

  private final runShooter50MotorCommand m_runShooter50MotorCommandAuto = new runShooter50MotorCommand(m_shooterMotorSubsystem, true);

  private final closeShooterPnumaticCommand m_closeShooterPnumaticCommandAuto = new closeShooterPnumaticCommand(m_shooterPnumaticSubsystem);

  private final shooterOnlyConveyorMotorCommand m_shooterOnlyConveyorMotorCommandAuto = new shooterOnlyConveyorMotorCommand(m_shooterIntakeSubsystem);

  private final ParallelDeadlineGroup m_TimedConveyorGroup = new ParallelDeadlineGroup(m_shooterConveyorTimerCommand, m_shooterOnlyConveyorMotorCommandAuto);

  private final SequentialCommandGroup m_WarmUpAndShootBalls = new SequentialCommandGroup(m_shooterWarmupTimerCommand, m_TimedConveyorGroup);

  private final ParallelDeadlineGroup m_CenterShootFromLine = new ParallelDeadlineGroup(m_WarmUpAndShootBalls, m_runShooter50MotorCommandAuto);

  private final TimerCommand m_driveBackwardsTimerAuto = new TimerCommand(500);

  private final drivetrainPercentPowerAuto m_drivetrainPercentPowerAuto = new drivetrainPercentPowerAuto(-.5, m_drivetrainSubsystem);

  private final ParallelDeadlineGroup m_driveBackwardsAndStop = new ParallelDeadlineGroup(m_driveBackwardsTimerAuto, m_drivetrainPercentPowerAuto);
  
  private final SequentialCommandGroup m_shootAndDriveBackwards = new SequentialCommandGroup(m_closeShooterPnumaticCommandAuto, m_CenterShootFromLine, m_driveBackwardsAndStop);

  //private final autonomusCommand2020 m_autonomusCommand = new autonomusCommand2020();

  //Wait 1 second.
  
  //private final ParallelDeadlineGroup m_intakefulltakeballParallel = new ParallelDeadlineGroup(m_ballObstructionSensorCommand, m_intakeTakeballMotorCommand);

  //private final conveyorbeltclearCommand m_ConveyorbeltclearCommand = new conveyorbeltclearCommand(m_shooterMotorSubsystem, m_bBallObstructionSensorSubsystem);

 // private final conveyorbeltObstructedCommand m_ConveyorbeltObstructedCommand = new conveyorbeltObstructedCommand(m_shooterMotorSubsystem, m_bBallObstructionSensorSubsystem);

  //private final SequentialCommandGroup m_shootallballs = new SequentialCommandGroup(m_closeShooterPnumaticCommand, m_ConveyorbeltclearCommand);

  //private final SequentialCommandGroup m_takeallballs = new SequentialCommandGroup(m_closeShooterPnumaticCommand, m_ConveyorbeltObstructedCommand);

  private final SequentialCommandGroup m_intakefulltakeball = new SequentialCommandGroup(m_openShooterPnumaticCommand, m_intakeTakeballMotorCommand);
  
  private final SequentialCommandGroup m_intakefullspitball = new SequentialCommandGroup(m_openShooterPnumaticCommand2, m_intakeSpitballMotorCommand);

  //private final SequentialCommandGroup m_auto = new SequentialCommandGroup(m_runShooter50MotorCommand, m_Wait500Command, m_shooterOnlyConveyorMotorCommand2, m_Wait2000Command, m_spinWheelMotorCommand, m_stopShooterMotorCommand);

  public static Object driveRobot;

  public static RobotContainer m_RobotContainer;

  public static Constants m_constants;

  public static XboxController Xbox1 = new XboxController(0);
  public static XboxController Xbox2 = new XboxController(1);
  public static Joystick Fightstick = new Joystick(2);
  
    //JoystickButton DriverA = new JoystickButton(Xbox1, XboxController.Button.kA.value); //Take Balls
    JoystickButton DriverA = new JoystickButton(Xbox1, XboxController.Button.kA.value); //Take Balls
    JoystickButton DriverB = new JoystickButton(Xbox1, XboxController.Button.kB.value); //Spit Balls
    JoystickButton DriverX = new JoystickButton(Xbox1, XboxController.Button.kX.value); //Opens pneumatic shooter
    JoystickButton DriverY = new JoystickButton(Xbox1, XboxController.Button.kY.value); //Closes pneumatic shooter
    JoystickButton DriverL = new JoystickButton(Xbox1, XboxController.Button.kBumperLeft.value);
    JoystickButton DriverR = new JoystickButton(Xbox1, XboxController.Button.kBumperRight.value);
    JoystickButton Driver2A = new JoystickButton(Xbox2, XboxController.Button.kA.value); 
    JoystickButton Driver2B = new JoystickButton(Xbox2, XboxController.Button.kB.value); 
    JoystickButton Driver2X = new JoystickButton(Xbox2, XboxController.Button.kX.value); 
    JoystickButton Driver2Y = new JoystickButton(Xbox2, XboxController.Button.kY.value); 
    JoystickButton Driver2L = new JoystickButton(Xbox2, XboxController.Button.kBumperLeft.value); 
    JoystickButton Driver2R = new JoystickButton(Xbox2, XboxController.Button.kBumperRight.value); 
    JoystickButton FightStickB = new JoystickButton(Fightstick, 3);
    JoystickButton FightStickY = new JoystickButton(Fightstick, 4);
    JoystickButton FightStickLB = new JoystickButton(Fightstick, 5);
    JoystickButton FightStickRB = new JoystickButton(Fightstick, 6);
    JoystickButton FightStickL3 = new JoystickButton(Fightstick, 9);
    JoystickButton FightStickR3 = new JoystickButton(Fightstick, 10);
    JoystickButton FightStickL1 = new JoystickButton(Fightstick, 5);
    //JoystickButton FightStickRT = new JoystickButton(Fightstick, 8);
    JoystickButton FightStickSHARE = new JoystickButton(Fightstick, 7);
    JoystickButton FightStickOPTIONS = new JoystickButton(Fightstick, 8);
    
    
    
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
    c_delevatorMotorCommand = new elevatorMotorCommand(Xbox2, m_elevatorMotorSubsystem);
    m_drivetrainSubsystem.setDefaultCommand(c_dDrivetrainCommand);
    m_elevatorMotorSubsystem.setDefaultCommand(c_delevatorMotorCommand);

    //m_constants.airCompressor = new Compressor(1);

    //m_constants.airCompressor.setClosedLoopControl(true);
    //m_constants.airCompressor.start();

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
    
    DriverA.whileHeld(m_runShooterMotorCommand);
    DriverB.whileHeld(m_stopShooterMotorCommand);
    DriverX.whileHeld(m_shooterOnlyConveyorMotorCommand);
    DriverY.whileHeld(m_runShooter50MotorCommand);
    DriverR.whileHeld(m_levelerLeftMotorCommand);
    DriverL.whileHeld(m_levelerRightMotorCommand);

    Driver2A.whileHeld(m_intakefulltakeball);
    Driver2B.whileHeld(m_intakefullspitball);
    Driver2X.whileHeld(m_spinWheelMotorCommand);
    //Driver2Y.toggleWhenPressed(m_kickoutReversePnumaticCommand);
    Driver2R.whenPressed(m_openShooterPnumaticCommand3);
    Driver2L.whenPressed(m_closeShooterPnumaticCommand2);
    
    //FightStickB.whenPressed(m_kickoutPnumaticCommand);
    FightStickY.whenPressed(m_climbarmupPnumaticCommand);
    FightStickRB.whenPressed(m_climbupPnumaticCommand);
    //FightStickRT.whenPressed(m_kickoutPnumaticCommand);
    FightStickL3.whenPressed(m_climbdownPnumaticCommand);
    //FightStickR3.whenPressed(m_climbdownPnumaticCommand);
    FightStickL1.whenPressed(m_climbarmdownPnumaticCommand);
    FightStickOPTIONS.whenPressed(m_kickoutPnumaticCommand);
    FightStickSHARE.whenPressed(m_kickoutReversePnumaticCommand);
    

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return (m_shootAndDriveBackwards);
    
  }
}
