package frc.robot.commands.utilityCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class ResetRobotCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;

    public ResetRobotCommand(
            IntakeSubsystem intakeSubsystem, 
            ElevatorSubsystem elevatorSubsystem, 
            HelicopterSubsystem helicopterSubsystem, 
            EndEffectorSubsystem endEffectorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;

        addRequirements(intakeSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(endEffectorSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
          Commands.runOnce(intakeSubsystem::stopIntake),
          Commands.runOnce(endEffectorSubsystem::stopEndEffector),
          new SequentialCommandGroup(
            elevatorSubsystem.clearIndexerPositionCommand(),
            new WaitCommand(0.6)
          ).onlyIf(() -> !helicopterSubsystem.isSafeForElevator()),
          helicopterSubsystem.stowPositionCommand(),
          intakeSubsystem.stowPositionCommand(),
          elevatorSubsystem.zeroElevatorCommand(),
          elevatorSubsystem.minPositionCommand()
        ).onlyIf(() -> elevatorSubsystem.getZeroed() && intakeSubsystem.getZeroed());
    }
}