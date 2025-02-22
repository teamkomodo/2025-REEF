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

public class newIntakeIndexCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;

    public newIntakeIndexCommand(
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
            //Commands.runOnce(() -> {
                new SequentialCommandGroup(
                    Commands.waitUntil(() -> (intakeSubsystem.isSafeForElevator())),
                    elevatorSubsystem.clearIntakePositionCommand(),
                    Commands.waitUntil(() -> elevatorSubsystem.atCommandedPosition()),
                    helicopterSubsystem.grabPositionCommand(),
                    Commands.waitUntil(() -> (helicopterSubsystem.atCommandedPosition())),
                    elevatorSubsystem.waitPositionCommand(),
                    Commands.waitUntil(() -> elevatorSubsystem.atCommandedPosition())
                ),//.schedule();
            //}),
            Commands.runOnce(() -> {intakeSubsystem.startIntake();  indexerSubsystem.startIndexer(); }),
            Commands.waitUntil(() -> (intakeSubsystem.getPieceInIntake())),
            Commands.runOnce(() -> intakeSubsystem.setDoneIntaking()),
            // Change the lights to indicate that the robot has a coral
            Commands.waitUntil(() -> (indexerSubsystem.getPieceIndexed())),
            Commands.runOnce(() -> {intakeSubsystem.stopIntake(); indexerSubsystem.stopIndexer();}),
            Commands.runOnce(() -> { intakeSubsystem.setDoneIntaking(); }),
            intakeSubsystem.clearCoralPositionCommand(),
            // Commands.waitSeconds(2),
            // //*/ Index
            Commands.runOnce(() -> {indexerSubsystem.startIndexer(); }), // bad reporting
            Commands.waitUntil(() -> (intakeSubsystem.atCommandedPosition() && helicopterSubsystem.atCommandedPosition())),
            elevatorSubsystem.clearIntakePositionCommand(),
            Commands.waitUntil(() -> elevatorSubsystem.atCommandedPosition()),
            // Commands.waitSeconds(2),
            // // Grab
            helicopterSubsystem.grabPositionCommand(),
            Commands.waitUntil(() -> (helicopterSubsystem.atCommandedPosition())),
            Commands.waitSeconds(0.5),
            elevatorSubsystem.waitPositionCommand(),
            elevatorSubsystem.grabPositionCommand(),
            endEffectorSubsystem.intakeCommand(),
            Commands.runOnce(() -> {indexerSubsystem.stopIndexer(); }), // bad reporting
            elevatorSubsystem.clearIntakePositionCommand(),
            Commands.waitUntil(() -> elevatorSubsystem.atCommandedPosition()),
            helicopterSubsystem.stowPositionCommand(),
            Commands.waitUntil(() -> (helicopterSubsystem.atCommandedPosition())),
            intakeSubsystem.stowPositionCommand(),
            elevatorSubsystem.stowPositionCommand(),

            // // Change the lights to indicate that the robot grabbed the coral and is now ready to score
            // // Stow
            // elevatorSubsystem.clearIntakePositionCommand(),
            // Commands.waitUntil(() -> (elevatorSubsystem.atCommandedPosition())),
            // helicopterSubsystem.stowPositionCommand(),
            // Commands.waitUntil(() -> (helicopterSubsystem.atCommandedPosition())),
            // elevatorSubsystem.stowPositionCommand()
            //*/
            
            Commands.runOnce(() -> {}) // Do nothing
        ).onlyIf(() -> (
            elevatorSubsystem.getZeroed() &&
            intakeSubsystem.getZeroed() && 
            !indexerSubsystem.getPieceInIndexer() &&
            !endEffectorSubsystem.getCoralLoaded()));
    }
}     