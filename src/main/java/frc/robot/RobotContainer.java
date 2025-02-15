// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeIndexCommand;
import frc.robot.commands.L1PositionCommand;
import frc.robot.commands.L2PositionCommand;
import frc.robot.commands.L3PositionCommand;
import frc.robot.commands.L4PositionCommand;
import frc.robot.commands.ScoreAndRemoveAlgaeCommand;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.ZeroElevatorCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.DRIVER_XBOX_PORT;
import static frc.robot.Constants.OPERATOR_XBOX_PORT;

import java.util.Dictionary;
import java.util.random.RandomGenerator.LeapableGenerator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  // private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
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


  private void configureBindings() {
    Trigger driverLT = driverController.leftTrigger();
    Trigger driverRT = driverController.rightTrigger();

    Trigger driverLB = driverController.leftBumper();
    Trigger driverRB = driverController.rightBumper();
    Trigger driverA = driverController.a();
    Trigger driverB = driverController.b();
    Trigger driverX = driverController.x();
    Trigger driverY = driverController.y();

    driverA.onTrue(intakeSubsystem.intakePositionCommand());
    driverB.onTrue(intakeSubsystem.stowPositionCommand());
    //driverX.onTrue(new IntakeIndexCommand(intakeSubsystem, null, elevatorSubsystem, helicopterSubsystem, null));
    driverY.onTrue(Commands.runOnce(() -> intakeSubsystem.startIntake()));
    driverY.onFalse(Commands.runOnce(() -> intakeSubsystem.stopIntake()));


    // Drivetrain commands
    driverLB.onTrue(drivetrainSubsystem.zeroGyroCommand());

    driverRT.onTrue(drivetrainSubsystem.enableSlowModeCommand());
    driverRT.onFalse(drivetrainSubsystem.disableSlowModeCommand());

    /* OFFICIAL CONTROLS */
    // driverLB.onTrue(new ScoreAndRemoveAlgaeCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem, drivetrainSubsystem));
    driverRB.onTrue(new ScoreCommand(endEffectorSubsystem, helicopterSubsystem, elevatorSubsystem, drivetrainSubsystem));
    driverLT.onTrue(new ZeroElevatorCommand(elevatorSubsystem, helicopterSubsystem));
    // driverRT.onTrue(new IntakeIndexCommand(intakeSubsystem, indexerSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem));
    driverX.onTrue(new L1PositionCommand(elevatorSubsystem, helicopterSubsystem));
    // driverY.onTrue(new L2PositionCommand(elevatorSubsystem, helicopterSubsystem));
    // driverB.onTrue(new L3PositionCommand(elevatorSubsystem, helicopterSubsystem));
    // driverA.onTrue(new L4PositionCommand(elevatorSubsystem, helicopterSubsystem));
    

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
