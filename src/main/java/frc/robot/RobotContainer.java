/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD licenseaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// FROM LARSON!! https://www.chiefdelphi.com/t/error-message-from-robotbase/162791/12

//Starting Position Angle 38 Degrees

// Robot is 158 incines from the cell port

package frc.robot;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.CenterOnTargetLimelight;
import frc.robot.commands.ShootWaitVelocity;
import frc.robot.commands.TimerCommand;
import frc.robot.commands.autonomusCommand2020;
import frc.robot.commands.ballObstructionSensorCommand;
import frc.robot.commands.checkForShooterVelocity;
import frc.robot.commands.climbArmDownPnumaticCommand;
import frc.robot.commands.climbArmUpPnumaticCommand;
import frc.robot.commands.climbHookExtendPnumaticCommand;
import frc.robot.commands.climbHookRetractPnumaticCommand;
import frc.robot.commands.closeShooterPnumaticCommand;
import frc.robot.commands.conveyorbeltObstructedCommand;
import frc.robot.commands.conveyorbeltclearCommand;
import frc.robot.commands.drivetrainCommand;
import frc.robot.commands.drivetrainPercentPowerAuto;
import frc.robot.commands.elevatorMotorCommand;
import frc.robot.commands.elevatorMoveToAngleMotorCommand;
import frc.robot.commands.intakeSpitballMotorCommand;
import frc.robot.commands.intakeTakeballMotorCommand;
import frc.robot.commands.kickoutPnumaticCommand;
import frc.robot.commands.kickoutReversePnumaticCommand;
import frc.robot.commands.levelerLeftMotorCommand;
import frc.robot.commands.levelerRightMotorCommand;
import frc.robot.commands.limelightAutoAimCommand;
import frc.robot.commands.openShooterPnumaticCommand;
import frc.robot.commands.runShooter50MotorCommand;
import frc.robot.commands.runShooterMotorCommand;
import frc.robot.commands.runUntilNotObstructedSensorCommand;
import frc.robot.commands.runUntilObstructedSensorCommand;
import frc.robot.commands.shooterOnlyConveyorMotorCommand;
import frc.robot.commands.shooterOnlyMotorCommand;
import frc.robot.commands.spinWheelMotorCommand;
import frc.robot.commands.stopConveyorMotorCommand;
import frc.robot.commands.stopShooterMotorCommand;
import frc.robot.commands.straightenballsCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IMUSubsystem;
import frc.robot.subsystems.LidarSubsystem;
import frc.robot.subsystems.ballObstructionSensorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.elevatorMotorSubsystem;
import frc.robot.subsystems.kickoutPnumaticSubsystem;
import frc.robot.subsystems.levelerMotorSubsystem;
import frc.robot.subsystems.shooterMotorSubsystem;
import frc.robot.subsystems.shooterIntakeSubsystem;
import frc.robot.subsystems.shooterPnumaticSubsystem;
import frc.robot.subsystems.wheelMotorSubsystem;
import frc.robot.subsystems.shooterMotorSubsystem.ShooterMotorStatus;
import frc.robot.Constants;
import frc.robot.subsystems.climbPnumaticSubsystem;
import frc.robot.subsystems.ballObstructionSensorSubsystem;
import frc.robot.commands.ballObstructionSensorCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.commands.limelightValuesCommand;
import edu.wpi.first.wpilibj.DriverStation;

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

  private static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
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

  private final LimeLightSubsystem s_limelightSubsystem = new LimeLightSubsystem();

  private final IMUSubsystem s_imuSubsystem = new IMUSubsystem();

  private final LidarSubsystem s_lidarSubsystem = new LidarSubsystem();

  // private final autonomusCommand2020 m_autonomusCommand2020 = new
  // autonomusCommand2020();

  private final WaitCommand m_Wait500Command = new WaitCommand(.5);

  private final WaitCommand m_Wait1000Command = new WaitCommand(1);

  private final WaitCommand m_Wait2000Command = new WaitCommand(2);

  private final kickoutPnumaticSubsystem m_kickoutPnumaticSubsystem = new kickoutPnumaticSubsystem();
  private final ballObstructionSensorSubsystem m_ballObstructionSensorSubsystem = new ballObstructionSensorSubsystem();
  private final wheelMotorSubsystem m_wheelMotorSubsystem = new wheelMotorSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final climbArmUpPnumaticCommand m_climbArmUpPnumaticCommand = new climbArmUpPnumaticCommand(
      m_climbPnumaticSubsystem);
  private final climbArmDownPnumaticCommand m_climbArmDownPnumaticCommand = new climbArmDownPnumaticCommand(
      m_climbPnumaticSubsystem);
  private final climbHookExtendPnumaticCommand m_climbHookExtendPnumaticCommand = new climbHookExtendPnumaticCommand(
      m_climbPnumaticSubsystem);
  private final climbHookRetractPnumaticCommand m_climbHookRetractPnumaticCommand = new climbHookRetractPnumaticCommand(
      m_climbPnumaticSubsystem);
  private final openShooterPnumaticCommand m_openShooterPnumaticCommand = new openShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);
  private final openShooterPnumaticCommand m_openShooterPnumaticCommand2 = new openShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);
  private final openShooterPnumaticCommand m_openShooterPnumaticCommand3 = new openShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);
  private final openShooterPnumaticCommand m_openShooterPnumaticCommandDriver1 = new openShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);

  private final openShooterPnumaticCommand m_openShooterPnumaticCommand4 = new openShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);

  private final openShooterPnumaticCommand m_openShooterPnumaticCommand5 = new openShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);

  private final closeShooterPnumaticCommand m_closeShooterPnumaticCommand = new closeShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);
  private final closeShooterPnumaticCommand m_closeShooterPnumaticCommand2 = new closeShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);

  private final closeShooterPnumaticCommand m_closeShooterPnumaticCommand3 = new closeShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);

  private final closeShooterPnumaticCommand m_closeShooterPnumaticCommand4 = new closeShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);    

  private final runShooterMotorCommand m_runShooterMotorCommand = new runShooterMotorCommand(m_shooterMotorSubsystem);

  private final runShooterMotorCommand m_runShooterMotorCommand2 = new runShooterMotorCommand(m_shooterMotorSubsystem);

  private final stopShooterMotorCommand m_stopShooterMotorCommand = new stopShooterMotorCommand(
      m_shooterMotorSubsystem);
  private final stopShooterMotorCommand m_KillMotorsDriver2 = new stopShooterMotorCommand(m_shooterMotorSubsystem);
  private final stopShooterMotorCommand m_KillMotorsDriver1 = new stopShooterMotorCommand(m_shooterMotorSubsystem);

  private final stopShooterMotorCommand m_stopShooterMotorCommand2 = new stopShooterMotorCommand(
      m_shooterMotorSubsystem);

  private final stopShooterMotorCommand m_stopShooterMotorCommand3 = new stopShooterMotorCommand(
      m_shooterMotorSubsystem);

  //private final stopConveyorMotorCommand m_stopConveyorMotorCommand = new stopConveyorMotorCommand(
      //m_shooterIntakeSubsystem);

      private final stopConveyorMotorCommand m_stopConveyorMotorCommand2 = new stopConveyorMotorCommand(
        m_shooterIntakeSubsystem);    

  private final shooterOnlyConveyorMotorCommand m_shooterOnlyConveyorMotorCommand2 = new shooterOnlyConveyorMotorCommand(
      m_shooterIntakeSubsystem);

  private final shooterOnlyConveyorMotorCommand m_shooterOnlyConveyorMotorCommand = new shooterOnlyConveyorMotorCommand(
      m_shooterIntakeSubsystem);

  private final shooterOnlyConveyorMotorCommand m_shooterOnlyConveyorMotorCommand3 = new shooterOnlyConveyorMotorCommand(m_shooterIntakeSubsystem);

  private final shooterOnlyConveyorMotorCommand m_shooterOnlyConveyorMotorCommand4 = new shooterOnlyConveyorMotorCommand(m_shooterIntakeSubsystem);

  private final shooterOnlyConveyorMotorCommand m_shooterOnlyConveyorMotorCommand_ConveyorCommandGroup = new shooterOnlyConveyorMotorCommand(m_shooterIntakeSubsystem);

  private final shooterOnlyMotorCommand m_shooterOnlyMotorCommand = new shooterOnlyMotorCommand(
      m_shooterMotorSubsystem);

  private final kickoutPnumaticCommand m_kickoutPnumaticCommand = new kickoutPnumaticCommand(
      m_kickoutPnumaticSubsystem);

  private final spinWheelMotorCommand m_spinWheelMotorCommand = new spinWheelMotorCommand(m_wheelMotorSubsystem);

  private final levelerLeftMotorCommand m_levelerLeftMotorCommand = new levelerLeftMotorCommand(
      m_levelerMotorSubsystem);

  private final levelerRightMotorCommand m_levelerRightMotorCommand = new levelerRightMotorCommand(
      m_levelerMotorSubsystem);

  private final ballObstructionSensorCommand m_BallObstructionSensorCommand = new ballObstructionSensorCommand(
      m_ballObstructionSensorSubsystem);

  private final kickoutReversePnumaticCommand m_kickoutReversePnumaticCommand = new kickoutReversePnumaticCommand(
      m_kickoutPnumaticSubsystem);

  private final intakeTakeballMotorCommand m_intakeTakeballMotorCommand = new intakeTakeballMotorCommand(
      m_shooterIntakeSubsystem, m_ballObstructionSensorSubsystem);

  private final intakeSpitballMotorCommand m_intakeSpitballMotorCommand = new intakeSpitballMotorCommand(
      m_shooterIntakeSubsystem);

  private final intakeSpitballMotorCommand m_intakeSpitballMotorCommandDriver1 = new intakeSpitballMotorCommand(
      m_shooterIntakeSubsystem);

  private final ShootWaitVelocity m_ShootWaitVelocity = new ShootWaitVelocity(m_shooterIntakeSubsystem,
      m_shooterMotorSubsystem, m_ballObstructionSensorSubsystem);

  private final closeShooterPnumaticCommand m_CloseBeforeShoot = new closeShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);

  private final ParallelCommandGroup m_CloseShootWaitVelocity = new ParallelCommandGroup(m_CloseBeforeShoot,
      m_ShootWaitVelocity);

  private final runShooter50MotorCommand m_runShooter50MotorCommand = new runShooter50MotorCommand(
      m_shooterMotorSubsystem, false);

  private final runShooter50MotorCommand m_runShooter50MotorCommand2 = new runShooter50MotorCommand(
      m_shooterMotorSubsystem, false);    

  private final limelightValuesCommand m_limelightValuesCommand = new limelightValuesCommand(s_limelightSubsystem);

  private final limelightAutoAimCommand m_limelightAutoAimCommand = new limelightAutoAimCommand(s_limelightSubsystem);

  private final runUntilObstructedSensorCommand m_runUntilObstructedSensorCommand = new runUntilObstructedSensorCommand(m_ballObstructionSensorSubsystem);

  private final runUntilObstructedSensorCommand m_runUntilObstructedSensorCommand2 = new runUntilObstructedSensorCommand(m_ballObstructionSensorSubsystem);

  private final runUntilObstructedSensorCommand m_runUntilObstructedSensorCommand3 = new runUntilObstructedSensorCommand(m_ballObstructionSensorSubsystem);

  private final runUntilObstructedSensorCommand m_runUntilObstructedSensorCommandGroup = new runUntilObstructedSensorCommand(m_ballObstructionSensorSubsystem);

  private final runUntilNotObstructedSensorCommand m_runUntilNotObstructedSensorCommand = new runUntilNotObstructedSensorCommand(m_ballObstructionSensorSubsystem);

  private final runUntilNotObstructedSensorCommand m_runUntilNotObstructedSensorCommandGroup = new runUntilNotObstructedSensorCommand(m_ballObstructionSensorSubsystem);

  private final checkForShooterVelocity m_checkForShooterVelocity = new checkForShooterVelocity(m_shooterMotorSubsystem);

  private final TimerCommand m_centerDriveBackCommand = new TimerCommand(1000);

  private final TimerCommand m_shooterWarmupTimerCommand = new TimerCommand(1000);

  // This is for autonomous to clear all three balls

  private final TimerCommand m_shooterConveyorTimerCommand = new TimerCommand(2000);

  private final runShooter50MotorCommand m_runShooter50MotorCommandAuto = new runShooter50MotorCommand(
      m_shooterMotorSubsystem, true);

  private final closeShooterPnumaticCommand m_closeShooterPnumaticCommandAuto = new closeShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);

  private final shooterOnlyConveyorMotorCommand m_shooterOnlyConveyorMotorCommandAuto = new shooterOnlyConveyorMotorCommand(
      m_shooterIntakeSubsystem);

  private final ParallelDeadlineGroup m_TimedConveyorGroup = new ParallelDeadlineGroup(m_shooterConveyorTimerCommand,
      m_shooterOnlyConveyorMotorCommandAuto);

  private final SequentialCommandGroup m_WarmUpAndShootBalls = new SequentialCommandGroup(m_shooterWarmupTimerCommand,
      m_TimedConveyorGroup);

  private final SequentialCommandGroup m_shoot50PercentAndCloseShooter = new SequentialCommandGroup (m_runShooter50MotorCommand, m_closeShooterPnumaticCommand4);  

  private final ParallelDeadlineGroup m_CenterShootFromLine = new ParallelDeadlineGroup(m_WarmUpAndShootBalls,
      m_runShooter50MotorCommandAuto);

  private final TimerCommand m_driveBackwardsTimerAuto = new TimerCommand(500);

  private final drivetrainPercentPowerAuto m_drivetrainPercentPowerAuto = new drivetrainPercentPowerAuto(-.5,
      m_drivetrainSubsystem);

  private final ParallelDeadlineGroup m_driveBackwardsAndStop = new ParallelDeadlineGroup(m_driveBackwardsTimerAuto,
      m_drivetrainPercentPowerAuto);

  private final SequentialCommandGroup m_moveConveyorUntilNotObstructed = new SequentialCommandGroup(m_shooterOnlyConveyorMotorCommand3, m_runUntilNotObstructedSensorCommand);

  private final SequentialCommandGroup m_moveConveyorUntilObstructed = new SequentialCommandGroup(m_shooterOnlyConveyorMotorCommand4, m_runUntilObstructedSensorCommand);

  private final SequentialCommandGroup m_shootAndDriveBackwards = new SequentialCommandGroup(
      m_closeShooterPnumaticCommandAuto, m_CenterShootFromLine, m_driveBackwardsAndStop);

  private final straightenballsCommand m_straightenballsCommand = new straightenballsCommand(m_shooterIntakeSubsystem);   

  private final closeShooterPnumaticCommand c_CloseShooterDriver2 = new closeShooterPnumaticCommand(
      m_shooterPnumaticSubsystem);

  private final SequentialCommandGroup m_stopAndCloseShooter = new SequentialCommandGroup(c_CloseShooterDriver2,
      m_stopShooterMotorCommand2);

  private final SequentialCommandGroup m_stopAndOpenShooter = new SequentialCommandGroup(m_openShooterPnumaticCommand5,
      m_stopShooterMotorCommand3);

  // private final autonomusCommand2020 m_autonomusCommand = new
  // autonomusCommand2020();

  // Wait 1 second.

   private final ParallelDeadlineGroup m_shootWaitObstructionParallel = new ParallelDeadlineGroup(m_runUntilObstructedSensorCommand3, new shooterOnlyConveyorMotorCommand(m_shooterIntakeSubsystem));

   private final ParallelDeadlineGroup m_runConveyorWithObstructionCheck = new ParallelDeadlineGroup(m_runUntilNotObstructedSensorCommandGroup, m_shooterOnlyConveyorMotorCommand_ConveyorCommandGroup);

   private final SequentialCommandGroup m_runConveyorWithObstructionAndVelocity = new SequentialCommandGroup(m_shootWaitObstructionParallel, m_checkForShooterVelocity, m_runConveyorWithObstructionCheck);

   private final conveyorbeltclearCommand m_ConveyorbeltclearCommand = new conveyorbeltclearCommand(m_shooterMotorSubsystem, m_ballObstructionSensorSubsystem);

  // private final conveyorbeltObstructedCommand m_ConveyorbeltObstructedCommand =
  // new conveyorbeltObstructedCommand(m_shooterMotorSubsystem,
  // m_bBallObstructionSensorSubsystem);

  private final SequentialCommandGroup m_shootallballs = new SequentialCommandGroup(m_closeShooterPnumaticCommand2, m_ConveyorbeltclearCommand);

  //private final SequentialCommandGroup m_waitUntilNoBalls = new SequentialCommandGroup(commands);

  private final SequentialCommandGroup m_takeallballs = new SequentialCommandGroup(m_closeShooterPnumaticCommand, m_runUntilObstructedSensorCommand2);

  private final SequentialCommandGroup m_intakefulltakeball = new SequentialCommandGroup(m_openShooterPnumaticCommand,
      m_intakeTakeballMotorCommand);

  private final SequentialCommandGroup m_intakefullspitball = new SequentialCommandGroup(m_openShooterPnumaticCommand2,
      m_intakeSpitballMotorCommand);
  private final SequentialCommandGroup m_intakefullspitballDriver1 = new SequentialCommandGroup(
      m_openShooterPnumaticCommandDriver1, m_intakeSpitballMotorCommandDriver1);

  // private final SequentialCommandGroup m_auto = new
  // SequentialCommandGroup(m_runShooter50MotorCommand, m_Wait500Command,
  // m_shooterOnlyConveyorMotorCommand2, m_Wait2000Command,
  // m_spinWheelMotorCommand, m_stopShooterMotorCommand);

  private final SequentialCommandGroup m_runShooterAndClosePnumatic = new SequentialCommandGroup(m_runShooterMotorCommand, m_closeShooterPnumaticCommand3);

  private final SequentialCommandGroup m_killPlayer2WithConveyor = new SequentialCommandGroup(m_KillMotorsDriver2, m_stopConveyorMotorCommand2);

  private final elevatorMoveToAngleMotorCommand c_ElevatorMoveToAngleMotorCommand = new elevatorMoveToAngleMotorCommand(m_elevatorMotorSubsystem, s_imuSubsystem, 24);

  public static Object driveRobot;

  public static RobotContainer m_RobotContainer;

  public static Constants m_constants;

  public static XboxController Xbox1 = new XboxController(0);
  public static XboxController Xbox2 = new XboxController(1);
  public static Joystick Fightstick = new Joystick(2);

  JoystickButton DriverA = new JoystickButton(Xbox1, XboxController.Button.kA.value); // Take Balls
  JoystickButton DriverB = new JoystickButton(Xbox1, XboxController.Button.kB.value); // Spit Balls
  JoystickButton DriverX = new JoystickButton(Xbox1, XboxController.Button.kX.value); // Opens pneumatic shooter
  JoystickButton DriverY = new JoystickButton(Xbox1, XboxController.Button.kY.value); // Closes pneumatic shooter
  JoystickButton DriverL = new JoystickButton(Xbox1, XboxController.Button.kBumperLeft.value);
  JoystickButton DriverR = new JoystickButton(Xbox1, XboxController.Button.kBumperRight.value);
  JoystickButton Driver1Start = new JoystickButton(Xbox1, XboxController.Button.kStart.value);
  // This is how you bind a trigger to a command - kind of confusing...
  Button DriveLeftTrigger = new Button(() -> Xbox1.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.5);
  Button DriveRightTrigger = new Button(() -> Xbox1.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5);

  JoystickButton Driver2A = new JoystickButton(Xbox2, XboxController.Button.kA.value);
  JoystickButton Driver2B = new JoystickButton(Xbox2, XboxController.Button.kB.value);
  JoystickButton Driver2X = new JoystickButton(Xbox2, XboxController.Button.kX.value);
  JoystickButton Driver2Y = new JoystickButton(Xbox2, XboxController.Button.kY.value);
  JoystickButton Driver2L = new JoystickButton(Xbox2, XboxController.Button.kBumperLeft.value);
  JoystickButton Driver2R = new JoystickButton(Xbox2, XboxController.Button.kBumperRight.value);
  JoystickButton Driver2Start = new JoystickButton(Xbox2, XboxController.Button.kStart.value);

  Button Drive2LeftTrigger = new Button(() -> Xbox2.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.5);
  Button Drive2RightTrigger = new Button(() -> Xbox2.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5);

  JoystickButton FightStickB = new JoystickButton(Fightstick, 3);
  JoystickButton FightStickY = new JoystickButton(Fightstick, 4);
  JoystickButton FightStickLB = new JoystickButton(Fightstick, 5);
  JoystickButton FightStickRB = new JoystickButton(Fightstick, 6);
  JoystickButton FightStickL3 = new JoystickButton(Fightstick, 9);
  JoystickButton FightStickR3 = new JoystickButton(Fightstick, 10);
  JoystickButton FightStickL1 = new JoystickButton(Fightstick, 5);
  JoystickButton FightStickSHARE = new JoystickButton(Fightstick, 7);
  JoystickButton FightStickOPTIONS = new JoystickButton(Fightstick, 8);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    setUpDrive();

    configureButtonBindings();
  }

  // Operations specific to TeleOp only
  public void teleopInit()
  {
    c_dDrivetrainCommand = new drivetrainCommand(Xbox1, m_drivetrainSubsystem);
    c_delevatorMotorCommand = new elevatorMotorCommand(Xbox2, m_elevatorMotorSubsystem);
    m_drivetrainSubsystem.setDefaultCommand(c_dDrivetrainCommand);
    m_elevatorMotorSubsystem.setDefaultCommand(c_delevatorMotorCommand);
    m_shooterMotorSubsystem.shooterMotorStatus = ShooterMotorStatus.IS_NOT_RUNNING;

     
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
    
    //DriverA.whileHeld(m_runShooterAndClosePnumatic);
    DriverB.whileHeld(m_stopShooterMotorCommand);
    DriverX.whileHeld(new CenterOnTargetLimelight(m_drivetrainSubsystem, s_limelightSubsystem));
    DriveLeftTrigger.whileHeld(m_intakefulltakeball);
    DriveRightTrigger.whileHeld(m_intakefullspitballDriver1);
    DriverY.whileHeld(m_straightenballsCommand);
    DriverR.whileHeld(m_levelerLeftMotorCommand);
    DriverL.whileHeld(m_levelerRightMotorCommand);
    Driver1Start.whenPressed(m_KillMotorsDriver1);

    Driver2A.whileHeld(m_runConveyorWithObstructionAndVelocity);
    //Driver2B.whileHeld(m_shootWaitObstructionParallel);
    //Driver2X.whileHeld(new CenterOnTargetLimelight(m_drivetrainSubsystem, s_limelightSubsystem));
    //Driver2Y.toggleWhenPressed(m_limelightAutoAimCommand);
    Driver2R.whenPressed(m_stopAndOpenShooter);
    Driver2L.whenPressed(m_stopAndCloseShooter);
    Driver2Start.whenPressed(m_killPlayer2WithConveyor);
    Drive2LeftTrigger.whileHeld(m_shoot50PercentAndCloseShooter);
    Drive2RightTrigger.whileHeld(m_runShooterAndClosePnumatic);
    
    //FightStickB.whenPressed(m_kickoutPnumaticCommand);
    FightStickY.whenPressed(m_climbHookExtendPnumaticCommand);
    FightStickRB.whenPressed(m_climbArmUpPnumaticCommand);
    //FightStickRT.whenPressed(m_kickoutPnumaticCommand);
    FightStickL3.whenPressed(m_climbArmDownPnumaticCommand);
    //FightStickR3.whenPressed(m_climbdownPnumaticCommand);
    FightStickL1.whenPressed(m_climbHookRetractPnumaticCommand);
    FightStickOPTIONS.whenPressed(c_ElevatorMoveToAngleMotorCommand);
    //FightStickSHARE.whileHeld(new CenterOnTargetLimelight(m_drivetrainSubsystem, s_limelightSubsystem));
    
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
