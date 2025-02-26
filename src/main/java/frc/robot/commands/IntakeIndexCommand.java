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
            Commands.runOnce(indexerSubsystem::allowIndexing),
            new SequentialCommandGroup(
                intakeSubsystem.intakePositionCommand(),
                new SequentialCommandGroup(
                    Commands.waitUntil(intakeSubsystem::isSafeForElevator),
                    elevatorSubsystem.clearIntakePositionCommand(),
                    Commands.waitUntil(elevatorSubsystem::atCommandedPosition),
                    helicopterSubsystem.grabPositionCommand(),
                    Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
                    elevatorSubsystem.waitPositionCommand(),
                    Commands.waitUntil(elevatorSubsystem::atCommandedPosition)
                ),
                Commands.runOnce(intakeSubsystem::startIntake),
                Commands.runOnce(indexerSubsystem::startIndexer),
                Commands.waitUntil(intakeSubsystem::getPieceInIntake),
                Commands.runOnce(intakeSubsystem::setDoneIntaking),
                Commands.waitUntil(indexerSubsystem::getPieceIndexed)
                // Change the lights to indicate that the robot has a coral
            ).onlyIf(() -> !indexerSubsystem.getPieceIndexed()),
            Commands.runOnce(intakeSubsystem::stopIntake),
            Commands.runOnce(intakeSubsystem::setDoneIntaking),
            intakeSubsystem.clearCoralPositionCommand(),
            // //*/ Index
            Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
            elevatorSubsystem.clearIntakePositionCommand(),
            Commands.waitUntil(() -> elevatorSubsystem.atCommandedPosition() && elevatorSubsystem.atCommandedPosition()),
            // // Grab
            helicopterSubsystem.grabPositionCommand(),
            Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
            Commands.waitSeconds(0.5),
            elevatorSubsystem.waitPositionCommand(),
            elevatorSubsystem.grabPositionCommand(),
            endEffectorSubsystem.intakeCommand(),
            Commands.runOnce(indexerSubsystem::stopIndexer),
            elevatorSubsystem.clearIntakePositionCommand(),
            Commands.waitUntil(elevatorSubsystem::atCommandedPosition),
            helicopterSubsystem.stowPositionCommand(),
            Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
            intakeSubsystem.stowPositionCommand(),
            elevatorSubsystem.stowPositionCommand()
        ).onlyIf(() -> (
            elevatorSubsystem.getZeroed() &&
            intakeSubsystem.getZeroed() &&
            !endEffectorSubsystem.getCoralLoaded()));
    }
}     