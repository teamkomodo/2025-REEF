package frc.robot.commands.intakeCommands;

import static frc.robot.Constants.INTAKE_HINGE_INTAKE_POSITION;
import static frc.robot.Constants.SLOW_INTAKE_SPEED;

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
import frc.robot.subsystems.LEDSubsystem;


public class IntakeToStowCommand extends DynamicCommand{
    
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final LEDSubsystem ledSubsystem;
   

    public IntakeToStowCommand(
        IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, 
        ElevatorSubsystem elevatorSubsystem, 
        HelicopterSubsystem helicopterSubsystem, 
        EndEffectorSubsystem endEffectorSubsystem,
        LEDSubsystem ledSubsystem) {
            this.intakeSubsystem = intakeSubsystem;
            this.indexerSubsystem = indexerSubsystem;
            this.elevatorSubsystem = elevatorSubsystem;
            this.helicopterSubsystem = helicopterSubsystem;
            this.endEffectorSubsystem = endEffectorSubsystem;
            this.ledSubsystem = ledSubsystem;
           

            addRequirements(intakeSubsystem);
            addRequirements(indexerSubsystem);
            addRequirements(elevatorSubsystem);
            addRequirements(helicopterSubsystem);
            addRequirements(endEffectorSubsystem);
            //addRequirements(ledSubsystem);
    }

    @Override
    protected Command getCommand() {
        // TODO Auto-generated method stub
        return new SequentialCommandGroup(

            elevatorSubsystem.preStowPositionCommand(),
            Commands.runOnce(() -> intakeSubsystem.setHingePosition(INTAKE_HINGE_INTAKE_POSITION)),
            Commands.runOnce(() -> {intakeSubsystem.startIntake();}),
            new WaitCommand(0.1),
            elevatorSubsystem.clearIndexerPositionCommand(),
            new WaitCommand(0.1),
            helicopterSubsystem.waitPositionCommand(),
            Commands.waitUntil(() -> !intakeSubsystem.coralIntakedSensor2.get() || indexerSubsystem.getPieceInIndexer()),
            elevatorSubsystem.waitPositionCommand(),
            Commands.waitUntil(indexerSubsystem::getPieceInIndexer),
            Commands.runOnce(intakeSubsystem::runSlow),
            Commands.waitUntil(indexerSubsystem::getPieceIndexed),
            intakeSubsystem.intakePositionCommand(),
            Commands.runOnce(intakeSubsystem::stopIntake),
            Commands.runOnce(intakeSubsystem::setDoneIntaking),
            new ParallelCommandGroup(
                Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(1)),
                helicopterSubsystem.grabPositionCommand()),
            new WaitCommand(0.3),
            elevatorSubsystem.grabPositionCommand(),
            new WaitCommand(0.3),
            Commands.runOnce(endEffectorSubsystem::stopEndEffector),
            new ParallelCommandGroup(
                elevatorSubsystem.preStowPositionCommand(),
                endEffectorSubsystem.securePiece()
            ),
            new WaitCommand(0.45),
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
        ).onlyIf(() -> !endEffectorSubsystem.getCoralLoaded());

        //throw new UnsupportedOperationException("Unimplemented method 'getCommand'");
    }

}
