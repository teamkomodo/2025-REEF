package frc.robot.commands.intakeCommands;

import static frc.robot.Constants.INTAKE_HINGE_INTAKE_POSITION;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeToStowCommand extends DynamicCommand{
    
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;

    public IntakeToStowCommand(
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
        // TODO Auto-generated method stub
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                elevatorSubsystem.preStowPositionCommand(),
                Commands.runOnce(() -> intakeSubsystem.setHingePosition(INTAKE_HINGE_INTAKE_POSITION)),
                Commands.runOnce(() -> {intakeSubsystem.startIntake();})),
            new WaitCommand(0.1),
            new ParallelCommandGroup(
                elevatorSubsystem.waitPositionCommand(),
                new SequentialCommandGroup(
                    new WaitCommand(0.1),
                    helicopterSubsystem.waitPositionCommand())),
            Commands.waitUntil(() -> !intakeSubsystem.coralIntakedSensor2.get()),
            elevatorSubsystem.prePickupPositionCommand(),
            intakeSubsystem.feedCoralPositionCommand(),
            Commands.waitUntil(indexerSubsystem::getPieceInIndexer),
            Commands.runOnce(intakeSubsystem::runSlow),
            Commands.waitUntil(indexerSubsystem::getPieceIndexed),
            new ParallelCommandGroup(
                Commands.runOnce(intakeSubsystem::stopIntake),
                Commands.runOnce(intakeSubsystem::setDoneIntaking),
                endEffectorSubsystem.startEndEffectorIntakingCommand(),
                elevatorSubsystem.grabPositionCommand(),
                helicopterSubsystem.grabPositionCommand()),
            Commands.waitUntil(endEffectorSubsystem::getCoralLoaded),
            Commands.runOnce(endEffectorSubsystem::stopEndEffector),
            elevatorSubsystem.preStowPositionCommand(),
            new WaitCommand(0.4),
            new ParallelCommandGroup(
                endEffectorSubsystem.securePiece(),
                helicopterSubsystem.stowPositionCommand(),
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    elevatorSubsystem.stowPositionCommand(),
                    endEffectorSubsystem.securePiece()
                ),
                intakeSubsystem.stowPositionCommand()
            )
        );

        //throw new UnsupportedOperationException("Unimplemented method 'getCommand'");
    }

}
