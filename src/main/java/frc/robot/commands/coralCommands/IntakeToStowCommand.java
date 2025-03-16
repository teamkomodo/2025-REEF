package frc.robot.commands.coralCommands;

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
            addRequirements(ledSubsystem);
    }

    @Override
    protected Command getCommand() {
        // TODO Auto-generated method stub
        return new SequentialCommandGroup(
            Commands.runOnce(() -> intakeSubsystem.setHingePosition(INTAKE_HINGE_INTAKE_POSITION)),
            elevatorSubsystem.preStowPositionCommand(),
            Commands.runOnce(() -> {intakeSubsystem.setIntakeDutyCycle(0.9);}),
            new WaitCommand(0.1),
            elevatorSubsystem.clearIndexerPositionCommand(),
            helicopterSubsystem.waitPositionCommand(),
            new WaitCommand(0.3),
            // Commands.waitUntil(() -> !intakeSubsystem.coralIntakedSensor2.get() || indexerSubsystem.getPieceInIndexer()),
            // //new ParallelCommandGroup(elevatorSubsystem.waitPositionCommand(),
            // //ledSubsystem.flashGreenCommand()),
            Commands.waitUntil(() -> !indexerSubsystem.coralInIndexerSensor.get() || !indexerSubsystem.coralIndexedSensor.get()),
            intakeSubsystem.feedCoralPositionCommand(),
            Commands.runOnce(() -> intakeSubsystem.setIntakeDutyCycle(0.4)),
            Commands.waitUntil(indexerSubsystem::getPieceIndexed),
            intakeSubsystem.intakePositionCommand(),
            Commands.runOnce(intakeSubsystem::stopIntake),
            Commands.runOnce(intakeSubsystem::setDoneIntaking),
            new ParallelCommandGroup(
                Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(1)),
                helicopterSubsystem.grabPositionCommand()),
            new WaitCommand(0.1),
            elevatorSubsystem.grabPositionCommand(),
            Commands.waitUntil(() -> !endEffectorSubsystem.coralLoadedSensor.get()),
            Commands.runOnce(endEffectorSubsystem::stopEndEffector),
            new ParallelCommandGroup(
                elevatorSubsystem.preStowPositionCommand(),
                endEffectorSubsystem.securePiece()),
            new WaitCommand(0.1),
            helicopterSubsystem.stowPositionCommand(),
            new WaitCommand(0.4),
            new ParallelCommandGroup(
                //ledSubsystem.flashBlueVioletCommand(),
                endEffectorSubsystem.securePiece(),
                new SequentialCommandGroup(
                    new WaitCommand(0.2),
                    elevatorSubsystem.stowPositionCommand(),
                    endEffectorSubsystem.securePiece()),
                intakeSubsystem.stowPositionCommand())
        ).onlyIf(() -> !endEffectorSubsystem.getCoralLoaded());
    }

}
