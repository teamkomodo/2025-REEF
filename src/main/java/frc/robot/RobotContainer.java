// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.algaeCommands.GrabFloorAlgaeCommand;
import frc.robot.commands.algaeCommands.KnockOutHighAlgaeCommand;
import frc.robot.commands.algaeCommands.KnockOutLowAlgaeCommand;
import frc.robot.commands.algaeCommands.ScoreAlgaeCommand;
import frc.robot.commands.intakeCommands.IntakeIndexCommand;
import frc.robot.commands.reefPositionCommands.L1PositionCommand;
import frc.robot.commands.reefPositionCommands.L2PositionCommand;
import frc.robot.commands.reefPositionCommands.L3PositionCommand;
import frc.robot.commands.reefPositionCommands.L4PositionCommand;
import frc.robot.commands.scoreCommands.CompleteScoreCommand;
import frc.robot.commands.utilityCommands.EjectCommand;
import frc.robot.commands.utilityCommands.IfElseCommand;
import frc.robot.commands.utilityCommands.ResetRobotCommand;
import frc.robot.commands.utilityCommands.ZeroMechCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

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

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  private void configureBindings() {
//Driver
    Trigger driverLT = driverController.leftTrigger();
    Trigger driverRT = driverController.rightTrigger();
    Trigger driverPD = driverController.povDown();
    Trigger driverPU = driverController.povUp();
    Trigger driverPL = driverController.povLeft();
    Trigger driverPR = driverController.povRight();

    Trigger driverLB = driverController.leftBumper();
    Trigger driverRB = driverController.rightBumper();
    Trigger driverA = driverController.a();
    Trigger driverB = driverController.b();
    Trigger driverX = driverController.x();
    Trigger driverY = driverController.y();

//Operator
    Trigger operatorLT = operatorController.leftTrigger();
    Trigger operatorRT = operatorController.rightTrigger();
    Trigger operatorPD = operatorController.povDown();
    Trigger operatorPU = operatorController.povUp();
    Trigger operatorPL = operatorController.povLeft();
    Trigger operatorPR = operatorController.povRight();

    Trigger operatorLB = operatorController.leftBumper();
    Trigger operatorRB = operatorController.rightBumper();
    Trigger operatorA = operatorController.a();
    Trigger operatorB = operatorController.b();
    Trigger operatorX = operatorController.x();
    Trigger operatorY = operatorController.y();

    /* Driver controls */
    /*  Button  | Command */
    /* driverLB | Enable / Disable Speed Mode  */
    /* driverRB | Enable / disable slow mode, Default is fast mode */
    /* driverX  | Zero gyro */
    /* driverY  | EMPTY */
    /* driverB  | Reset robot */
    /* driverLT | Go to left branch */
    /* driverRT | Go to right branch */
    /* driver left joystick | Drive the robot */
    /* driver right joysticks | Rotate the robot */


    /* Operator controls */
    /*   Button   | Command */
    /* operatorLT | Score coral */
    /* operatorRT | Intake and index */
    /* operatorRB | Score algae */
    /* operatorLB | Zero elevator and intake, or reset robot mechs, including zero elevator */
    /* operatorPD (POV down) | Eject from intake */
    /* operatorPL (POV left) | Knock out low algae */
    /* operatorPU (POV up) | Knock out high algae */
    /* operatorPR (POV right) | _Grab_ ground algae */
    /* operatorX  | L1 Position */
    /* operatorY  | L2 Position */
    /* operatorA  | L4 Position */
    /* operatorB  | L3 Position */

    
    operatorLB.onTrue(new IfElseCommand(
      () -> (elevatorSubsystem.getZeroed() && intakeSubsystem.getZeroed()),
      new ResetRobotCommand(intakeSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem),
      new ZeroMechCommand(elevatorSubsystem, intakeSubsystem, helicopterSubsystem)
    ));
    operatorLT.onTrue(new CompleteScoreCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem));
    operatorRT.onTrue(new IntakeIndexCommand(intakeSubsystem, indexerSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));
    operatorPD.onTrue(new EjectCommand(intakeSubsystem, endEffectorSubsystem));

    // Algae commands
    operatorPL.onTrue(new KnockOutLowAlgaeCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem));
    operatorPU.onTrue(new KnockOutHighAlgaeCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem));
    operatorPR.onTrue(new GrabFloorAlgaeCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem));
    operatorRB.onTrue(new ScoreAlgaeCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem));

    // Level position commands
    operatorX.onTrue(new L1PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));
    operatorY.onTrue(new L2PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));
    operatorB.onTrue(new L3PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));
    operatorA.onTrue(new L4PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));


    driverX.onTrue(drivetrainSubsystem.zeroGyroCommand());
    driverLB.onTrue(drivetrainSubsystem.disableSpeedModeCommand());
    driverLB.onFalse(drivetrainSubsystem.enableSpeedModeCommand());

    driverLT.onTrue(drivetrainSubsystem.goToBranch(false));
    driverRT.onTrue(drivetrainSubsystem.goToBranch(true));
    driverRB.whileTrue(drivetrainSubsystem.limelightAlignCommand());
    // FIXME: Remove this, it is just for convenience during auto testing
    driverB.onTrue(new ResetRobotCommand(intakeSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));
    driverA.onTrue(drivetrainSubsystem.driveSysIdRoutineCommand());
    
    // deadband and curves are applied in command
    drivetrainSubsystem.setDefaultCommand(
      drivetrainSubsystem.joystickDriveCommand(
        () -> ( -driverController.getLeftY() ), // -Y on left joystick is +X for robot
        () -> ( -driverController.getLeftX() ), // -X on left joystick is +Y for robot
        () -> ( driverController.getRightX() ) // -X on right joystick is +Z for robot
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Reset", new ResetRobotCommand(intakeSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));
    NamedCommands.registerCommand("L4", new L4PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));
    NamedCommands.registerCommand("Score", new CompleteScoreCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem));
    NamedCommands.registerCommand("Zero", drivetrainSubsystem.zeroGyroCommand());
  }
  public Command getAutonomousCommand() {
    // // An example command will be run in autonomous
    if(autoChooser != null){
      return autoChooser.getSelected();
    }

    return null;
    //return autoChooser.getSelected();
    //return AutoBuilder.buildAuto("Leave");//(PathPlannerPath.fromPathFile("Leave.path"));//new PathPlannerAuto("Leave");

  
  }
}
