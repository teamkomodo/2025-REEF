// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.coralCommands.EjectCommand;
import frc.robot.commands.coralCommands.IntakeToStowCommand;
import frc.robot.commands.coralCommands.ScoreToStowCommand;
import frc.robot.commands.reefPositionCommands.L2PositionCommand;
import frc.robot.commands.reefPositionCommands.L3PositionCommand;
import frc.robot.commands.reefPositionCommands.L4PositionCommand;
import frc.robot.commands.coralCommands.AlignToBranchCommand;
import frc.robot.commands.resetCommands.ResetRobotCommand;
import frc.robot.commands.resetCommands.ZeroMechCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import static frc.robot.Constants.DRIVER_XBOX_PORT;
import static frc.robot.Constants.OPERATOR_XBOX_PORT;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  //Inputs Devices
  private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT); 
  private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_XBOX_PORT);

  //Subsystems
  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final HelicopterSubsystem helicopterSubsystem = new HelicopterSubsystem();
  private final EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {

    /*           Driver | Control
     *        Joysticks | Drive
     *         X Button | Zero Gyro
     *      Left Bumper | Slow Mode
     *     Right Bumper | Reef Align
     *          Trigger | Branch Align & Score
     * 
     *         Operator | Control
     *     Right Bumper | Zero Robot
     *      Left Bumper | Stop & Stow Robot
     *    Right Trigger | Intake
     *         A Button | L4 Position
     *         B Button | L3 Position
     *         Y Button | L2 Position
     *     Left Trigger | Score
     *         POV Down | Eject
     */

    Trigger driverLT = driverController.leftTrigger();
    Trigger driverRT = driverController.rightTrigger();

    Trigger driverLB = driverController.leftBumper();
    Trigger driverRB = driverController.rightBumper();
    Trigger driverX = driverController.x();

    Trigger operatorRT = operatorController.rightTrigger();
    Trigger operatorLT = operatorController.leftTrigger();
    Trigger operatorPD = operatorController.povDown();

    Trigger operatorLB = operatorController.leftBumper();
    Trigger operatorRB = operatorController.rightBumper();
    Trigger operatorA = operatorController.a();

    Trigger operatorB = operatorController.b();
    Trigger operatorY = operatorController.y();

    operatorRT.onTrue(new IntakeToStowCommand(intakeSubsystem, indexerSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem, ledSubsystem));
    operatorPD.onTrue(new EjectCommand(intakeSubsystem, endEffectorSubsystem, ledSubsystem, helicopterSubsystem, elevatorSubsystem));

    operatorRB.onTrue(new ZeroMechCommand(elevatorSubsystem, intakeSubsystem, helicopterSubsystem, ledSubsystem));
    operatorLB.onTrue(new ResetRobotCommand(intakeSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem, ledSubsystem));

    // operatorLT.onTrue(helicopterSubsystem.l4WaitPositionCommand());
    // operatorRT.onTrue(helicopterSubsystem.grabPositionCommand());
    operatorA.onTrue(new L4PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));
    operatorB.onTrue(new L3PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));
    operatorY.onTrue(new L2PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));

    //operatorY.onTrue(intakeSubsystem.intakePositionCommand());
    //operatorA.onTrue(elevatorSubsystem.l2PositionCommand());
    //operatorB.onTrue(elevatorSubsystem.l4PositionCommand());
    driverX.onTrue(drivetrainSubsystem.zeroGyroCommand());
    driverLB.onTrue(drivetrainSubsystem.disableSpeedModeCommand());
    driverLB.onFalse(drivetrainSubsystem.enableSpeedModeCommand());
    driverRT.whileTrue(drivetrainSubsystem.goToBranch(true));
    driverLT.whileTrue(new AlignToBranchCommand(drivetrainSubsystem, endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem, intakeSubsystem, true, ledSubsystem));
    driverRB.whileTrue(drivetrainSubsystem.limelightAlignCommand());
    //driverRT.onTrue(new ScoreToStowCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem, intakeSubsystem, ledSubsystem));
    
    // deadband and curves are applied in command
    drivetrainSubsystem.setDefaultCommand(
      drivetrainSubsystem.joystickDriveCommand(
        () -> ( driverController.getLeftY() ), // -Y on left joystick is +X for robot
        () -> ( driverController.getLeftX() ), // -X on left joystick is +Y for robot
        () -> ( driverController.getRightX() ) // -X on right joystick is +Z for robot
      )
    );
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Reset", new ResetRobotCommand(intakeSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem, ledSubsystem));
    NamedCommands.registerCommand("L4", new L4PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));
    NamedCommands.registerCommand("Score", new ScoreToStowCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem, intakeSubsystem, ledSubsystem));
    NamedCommands.registerCommand("Zero", drivetrainSubsystem.zeroGyroCommand());
    NamedCommands.registerCommand("Reset", new ResetRobotCommand(intakeSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem, ledSubsystem));
    NamedCommands.registerCommand("StowArm", helicopterSubsystem.stowPositionCommand());
    NamedCommands.registerCommand("Intake", new IntakeToStowCommand(intakeSubsystem, indexerSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem, ledSubsystem));
  }

  public void teleopInit
  () {
    new ResetRobotCommand(intakeSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem, ledSubsystem).schedule();
  }

  public Command getAutonomousCommand() {
    // // An example command will be run in autonomous
    if(autoChooser != null)
      return autoChooser.getSelected();
    return null;
  }
  }




