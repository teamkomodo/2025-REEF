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
            intakeSubsystem.clearArmPositionCommand(),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    intakeSubsystem.intakePositionCommand(),
                    Commands.runOnce(intakeSubsystem::setDoneIntaking),
                    Commands.waitUntil(intakeSubsystem::atCommandedPosition),
                    Commands.runOnce(() -> intakeSubsystem.setHingeDutyCycle(0)),
                    Commands.runOnce(() -> intakeSubsystem.startIntake()),
                    Commands.waitUntil(intakeSubsystem::getPieceInIntake),
                    intakeSubsystem.feedCoralPositionCommand(),
                    Commands.waitUntil(indexerSubsystem::getPieceInIndexer),
                    Commands.runOnce(intakeSubsystem::runSlow),
                    Commands.waitUntil(indexerSubsystem::getPieceIndexed),
                    Commands.runOnce(intakeSubsystem::setDoneIntaking),
                    Commands.runOnce(intakeSubsystem::stopIntake),
                    Commands.runOnce(intakeSubsystem::setDoneIntaking)
                ).onlyIf(() -> !indexerSubsystem.getPieceIndexed()),
                new SequentialCommandGroup(
                    Commands.waitUntil(intakeSubsystem::isSafeForElevator),
                    elevatorSubsystem.waitPositionCommand(),
                    Commands.waitUntil(elevatorSubsystem::atCommandedPosition),
                    helicopterSubsystem.grabPositionCommand(),
                    Commands.waitUntil(helicopterSubsystem::atCommandedPosition)
                )
            ),
            Commands.runOnce(intakeSubsystem::stopIntake),
            Commands.runOnce(intakeSubsystem::setDoneIntaking),
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
            elevatorSubsystem.clearIndexerPositionCommand(),
            Commands.waitUntil(elevatorSubsystem::aboveCommandedPosition),
            helicopterSubsystem.stowPositionCommand(),
            Commands.runOnce(intakeSubsystem::setDoneIntaking),
            intakeSubsystem.feedCoralPositionCommand(),
            Commands.waitUntil(() -> (helicopterSubsystem.atCommandedPosition() || helicopterSubsystem.isSafeForElevator())),
            intakeSubsystem.stowPositionCommand(),
            new SequentialCommandGroup(
                endEffectorSubsystem.startEndEffectorIntakingCommand(),
                Commands.waitUntil(endEffectorSubsystem::getCoralLoaded),
                Commands.runOnce(endEffectorSubsystem::stopEndEffector)
            ).onlyIf(() -> !endEffectorSubsystem.getCoralLoaded()),
            Commands.runOnce(() -> {})
        ).onlyIf(() -> (elevatorSubsystem.getZeroed() &&
                intakeSubsystem.getZeroed() &&
                !endEffectorSubsystem.getCoralLoaded()));
    }
}