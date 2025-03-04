// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.intakeCommands.IntakeIndexCommand;
import frc.robot.commands.reefPositionCommands.L1PositionCommand;
import frc.robot.commands.reefPositionCommands.L2PositionCommand;
import frc.robot.commands.reefPositionCommands.L3PositionCommand;
import frc.robot.commands.reefPositionCommands.L4PositionCommand;
import frc.robot.commands.scoreCommands.CompleteScoreCommand;
import frc.robot.commands.scoreCommands.ScoreAndRemoveAlgaeCommand;
import frc.robot.commands.scoreCommands.ScoreCommand;
import frc.robot.commands.scoreCommands.StartScoreCommand;
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

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  boolean scoreStarted = false;
  private void configureBindings() {
    Trigger driverRB = driverController.rightBumper();
    Trigger driverLB = driverController.leftBumper();

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
    /* driverLB  | Zero gyro */
    /* driverRB | Disable / enable slow mode, Default is slow */
    /* driver left joystick | Drive the robot */
    /* driver right joysticks | Rotate the robot */


    /* Operator controls */
    /*   Button   | Command */
    /* operatorLB | Zero elevator and intake, or reset robot mechs, including zero elevator */
    /* operatorRB | Score and remove algae */
    /* operatorLT | Score */
    /* operatorRT | Intake and index */
    /* operatorPD (POV down) | Eject from intake */
    /* operatorPU (POV up) | Intake up */
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

    // operatorPU.onTrue(new IfElseCommand(
    //   () -> (scoreStarted),
    //   new SequentialCommandGroup(
    //     Commands.runOnce(() -> { scoreStarted = false; }),
    //     new ScoreCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem)
    //   ),
    //   new SequentialCommandGroup(
    //     new StartScoreCommand(helicopterSubsystem, elevatorSubsystem),
    //     Commands.runOnce(() -> { scoreStarted = true; })
    //   )
    // ));

    operatorRB.onTrue(new ScoreAndRemoveAlgaeCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem));

    operatorRT.onTrue(new IntakeIndexCommand(intakeSubsystem, indexerSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));

    operatorPU.onTrue(
      new SequentialCommandGroup(
        Commands.runOnce(() -> intakeSubsystem.stopIntake()), 
        intakeSubsystem.stowPositionCommand()
    ));
    
    operatorPD.onTrue(new SequentialCommandGroup(
      Commands.runOnce(() -> intakeSubsystem.reverseIntake()),
      Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(-0.7)),
      Commands.waitSeconds(0.6),
      Commands.runOnce(intakeSubsystem::stopIntake),
      Commands.runOnce(endEffectorSubsystem::stopEndEffector)
    ));

    

    operatorX.onTrue(new SequentialCommandGroup(
      new L1PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem),
      Commands.runOnce(() -> { scoreStarted = false; })
    ));

    operatorY.onTrue(new SequentialCommandGroup(
      new L2PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem),
      Commands.runOnce(() -> { scoreStarted = false; })
    ));

    operatorB.onTrue(new SequentialCommandGroup(
      new L3PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem),
      Commands.runOnce(() -> { scoreStarted = false; })
    ));

    operatorA.onTrue(new SequentialCommandGroup(
      new L4PositionCommand(elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem),
      Commands.runOnce(() -> { scoreStarted = false; })
    ));


    driverLB.onTrue(drivetrainSubsystem.zeroGyroCommand());
    driverRB.onTrue(drivetrainSubsystem.disableSpeedModeCommand());
    driverRB.onFalse(drivetrainSubsystem.enableSpeedModeCommand());
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
  public Command getAutonomousCommand() {
    // // An example command will be run in autonomous
    // if(autoChooser != null){
    //   return autoChooser.getSelected();
    //   System.out.println("got an auto");
    // }

    // return null;

    return autoChooser.getSelected();
    // return new PathPlannerAuto("Test Path");

  
  }
}
