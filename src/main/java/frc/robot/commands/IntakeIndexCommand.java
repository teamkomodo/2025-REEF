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
        addRequirements(elevatorSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(endEffectorSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            // Intake
            Commands.runOnce(() -> indexerSubsystem.allowIndexing()),
            intakeSubsystem.intakePositionCommand(),
            Commands.runOnce(() -> intakeSubsystem.startIntake()),
            Commands.waitUntil(() -> (intakeSubsystem.getPieceInIntake())),
            Commands.runOnce(() -> intakeSubsystem.setDoneIntaking()),
            // Change the lights to indicate that the robot has a coral
            Commands.waitUntil(() -> (indexerSubsystem.getPieceFullyIntaked())),
            Commands.runOnce(() -> intakeSubsystem.stopIntake())//,
            // //*/ Index
            // intakeSubsystem.stowPositionCommand(),
            // elevatorSubsystem.waitPositionCommand(),
            // Commands.runOnce(() -> { intakeSubsystem.setDoneIntaking(); }),
            // Commands.waitUntil(() -> indexerSubsystem.getPieceIndexed()),
            // Commands.runOnce(() -> indexerSubsystem.disallowIndexing()),
            // // Grab
            // helicopterSubsystem.grabPositionCommand(),
            // Commands.waitUntil(() -> (helicopterSubsystem.atCommandedPosition())),
            // elevatorSubsystem.grabPositionCommand(),
            // endEffectorSubsystem.intakeCommand(),
            // // Change the lights to indicate that the robot grabbed the coral and is now ready to score
            // // Stow
            // elevatorSubsystem.clearIntakePositionCommand(),
            // Commands.waitUntil(() -> (elevatorSubsystem.atCommandedPosition())),
            // helicopterSubsystem.stowPositionCommand(),
            // Commands.waitUntil(() -> (helicopterSubsystem.atCommandedPosition())),
            // elevatorSubsystem.stowPositionCommand()
            //*/
        ).onlyIf(() -> (
            elevatorSubsystem.getZeroed() &&
            intakeSubsystem.getZeroed() && 
            !indexerSubsystem.getPieceInIndexer() &&
            !endEffectorSubsystem.getCoralLoaded()));
    }
}     