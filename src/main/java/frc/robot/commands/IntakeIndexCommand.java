package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeIndexCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;

    public IntakeIndexCommand(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(intakeSubsystem);
        addRequirements(indexerSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> indexerSubsystem.allowIndexing()),
            Commands.runOnce(() -> intakeSubsystem.startIntake()),
            Commands.waitUntil(() -> (indexerSubsystem.getPieceInIndexer())),
            Commands.runOnce(() -> intakeSubsystem.stopIntake()),
            Commands.runOnce(() -> { intakeSubsystem.setDoneIntaking(); }),
            Commands.runOnce(() -> indexerSubsystem.disallowIndexing())
        );
    }
}     