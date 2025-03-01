package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeIndexCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;

    public IntakeIndexCommand(
            IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, 
            ElevatorSubsystem elevatorSubsystem, 
            HelicopterSubsystem helicopterSubsystem, 
            EndEffectorSubsystem endEffectorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;

        addRequirements(intakeSubsystem);
        addRequirements(indexerSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(endEffectorSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            // Intake and index
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    intakeSubsystem.intakePositionCommand(),
                    Commands.waitUntil(intakeSubsystem::atCommandedPosition),
                    Commands.runOnce(intakeSubsystem::startIntake),
                    Commands.waitUntil(intakeSubsystem::getPieceInIntake),
                    Commands.waitUntil(indexerSubsystem::getPieceIndexed)
                ).onlyIf(() -> !indexerSubsystem.getPieceIndexed()),
                new SequentialCommandGroup(
                    Commands.waitUntil(intakeSubsystem::isSafeForElevator),
                    elevatorSubsystem.clearIntakePositionCommand(),
                    Commands.waitUntil(elevatorSubsystem::atCommandedPosition),
                    helicopterSubsystem.grabPositionCommand(),
                    Commands.waitUntil(helicopterSubsystem::atCommandedPosition)
                )
            ),
            Commands.runOnce(intakeSubsystem::stopIntake),
            Commands.runOnce(intakeSubsystem::setDoneIntaking),
            intakeSubsystem.clearCoralPositionCommand(),
            new ParallelRaceGroup(
                new RepeatCommand(
                    new SequentialCommandGroup(
                        Commands.waitUntil(indexerSubsystem::getPieceIndexed),
                        elevatorSubsystem.grabPositionCommand(),
                        Commands.waitUntil(() -> !indexerSubsystem.getPieceIndexed()),
                        elevatorSubsystem.waitPositionCommand()
                    )
                ),
                endEffectorSubsystem.intakeCommand() // Waits until it has the coral, at which point the other command will be interrupted
            ),
            elevatorSubsystem.clearIntakePositionCommand(),
            Commands.waitUntil(elevatorSubsystem::atCommandedPosition),
            helicopterSubsystem.stowPositionCommand(),
            Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
            intakeSubsystem.stowPositionCommand(),
            elevatorSubsystem.minPositionCommand(),
            Commands.waitUntil(() -> elevatorSubsystem.atCommandedPosition()),
            elevatorSubsystem.zeroElevatorCommand(), //*/
            Commands.runOnce(() -> {})
        ).onlyIf(() -> (elevatorSubsystem.getZeroed() &&
                intakeSubsystem.getZeroed() &&
                !endEffectorSubsystem.getCoralLoaded()));
    }
}     