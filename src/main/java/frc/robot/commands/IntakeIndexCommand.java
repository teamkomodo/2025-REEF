package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.IntakeSubsystem;
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
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            // Intake
            Commands.runOnce(() -> indexerSubsystem.allowIndexing()),
            intakeSubsystem.intakePositionCommand(),
            Commands.runOnce(() -> intakeSubsystem.startIntake()),
            Commands.waitUntil(() -> (indexerSubsystem.getPieceInIndexer())),
            Commands.runOnce(() -> intakeSubsystem.stopIntake())//,
            /*/ Index
            intakeSubsystem.stowPositionCommand(),
            elevatorSubsystem.waitPositionCommand(),
            Commands.runOnce(() -> { intakeSubsystem.setDoneIntaking(); }),
            Commands.waitUntil(() -> indexerSubsystem.getPieceIndexed()),
            Commands.runOnce(() -> indexerSubsystem.disallowIndexing()),
            // Grab
            helicopterSubsystem.grabPositionCommand(),
            Commands.waitUntil(() -> (helicopterSubsystem.atCommandedPosition())),
            elevatorSubsystem.grabPositionCommand(),
            endEffectorSubsystem.intakeCommand()
            //*/
        );
    }
}     