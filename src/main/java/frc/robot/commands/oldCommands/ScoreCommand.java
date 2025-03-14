package frc.robot.commands.oldCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ScoreCommand extends DynamicCommand {

    private final EndEffectorSubsystem endEffectorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public ScoreCommand(
                EndEffectorSubsystem endEffectorSubsystem, HelicopterSubsystem helicopterSubsystem, 
                ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(endEffectorSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(intakeSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            // Release
            intakeSubsystem.stowPositionCommand(),
            endEffectorSubsystem.ejectCommand(),
            helicopterSubsystem.releaseCoralPositionCommand(),
            // // Return to waiting position
            Commands.waitSeconds(0.1),
            Commands.waitUntil(intakeSubsystem::atCommandedPosition),
            elevatorSubsystem.clearIndexerPositionCommand(),
            Commands.waitUntil(elevatorSubsystem::aboveCommandedPosition),
            helicopterSubsystem.waitPositionCommand(),
            elevatorSubsystem.waitPositionCommand(),
            Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
            Commands.waitUntil(elevatorSubsystem::atCommandedPosition)
        ).onlyIf(() -> (elevatorSubsystem.getZeroed() && helicopterSubsystem.getPositionWaitingOn() != 0));
    }
}