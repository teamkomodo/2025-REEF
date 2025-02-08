// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeIndexCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.Constants.DRIVER_XBOX_PORT;
import static frc.robot.Constants.OPERATOR_XBOX_PORT;

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
  // private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }


  private void configureBindings() {
    Trigger driverRT = driverController.rightTrigger();

    driverRT.onTrue(drivetrainSubsystem.enableSlowModeCommand());
    driverRT.onFalse(drivetrainSubsystem.disableSlowModeCommand());

    Trigger driverLB = driverController.leftBumper();
    driverLB.onTrue(drivetrainSubsystem.zeroGyroCommand());

    Trigger driverRB = driverController.leftBumper();
    //driverRB.onTrue(intakeSubsystem.unzeroCommand());


    
    Trigger driverA = driverController.a();
    Trigger driverB = driverController.b();
    Trigger driverX = driverController.x();
    Trigger driverY = driverController.y();
    // driverB.onTrue(new IntakeIndexCommand(intakeSubsystem, indexerSubsystem));
    // driverA.whileTrue(intakeSubsystem.zeroHingeCommand());
    // driverX.onTrue(intakeSubsystem.stationIntakePositionCommand());
    // driverY.onTrue(intakeSubsystem.intakePositionCommand());
    // driverY.onTrue(Commands.runOnce(() -> intakeSubsystem.setHingeDutyCycle(0)));
    driverA.onTrue(elevatorSubsystem.zeroElevatorCommand());
    driverB.onTrue(elevatorSubsystem.stowPositionCommand());
    driverX.onTrue(elevatorSubsystem.l2PositionCommand());
    driverY.onTrue(elevatorSubsystem.l4PositionCommand());
    

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
