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
            Commands.print("GOING TO INTAKE POSITON"),
            Commands.runOnce(() -> intakeSubsystem.setHingePosition(INTAKE_HINGE_INTAKE_POSITION)),
            Commands.waitSeconds(1),
            Commands.print("ELEVATOR PRESTOWING"),
            elevatorSubsystem.preStowPositionCommand(),
            Commands.runOnce(() -> {intakeSubsystem.setIntakeDutyCycle(0.5);}),
            new WaitCommand(0.4), //0.4
            Commands.print("ELEVATOR CLEARING INDEXER"),
            elevatorSubsystem.clearIndexerPositionCommand(),
            Commands.print("HELICOPTER WAITING"),
            helicopterSubsystem.waitPositionCommand(),
            new WaitCommand(1),
            Commands.runOnce(() -> intakeSubsystem.setHingeDutyCycle(0)),
            Commands.waitUntil(() -> !intakeSubsystem.coralIntakedSensor2.get()/*  || indexerSubsystem.getPieceInIndexer()*/),
            Commands.print("ELEVATOR GOING TO WAIT POSITION"),
            elevatorSubsystem.waitPositionCommand(),
            // //ledSubsystem.flashGreenCommand()),
            //Commands.waitSeconds(2),
            Commands.waitUntil(() -> intakeSubsystem.coralIntakedSensor.get()),
            //Commands.waitUntil(() -> !indexerSubsystem.coralInIndexerSensor.get() || !indexerSubsystem.coralIndexedSensor.get()),
            Commands.print("INTAKE FEEDING CORAL"),
            intakeSubsystem.feedCoralPositionCommand(),
            Commands.runOnce(() -> intakeSubsystem.setIntakeDutyCycle(0.5)),
            Commands.waitSeconds(1.5),
            //Commands.waitUntil(indexerSubsystem::getPieceIndexed),
            Commands.print("GOING TO INTAKE POSITION"),
            intakeSubsystem.intakePositionCommand(),
            Commands.runOnce(intakeSubsystem::stopIntake),
            Commands.runOnce(intakeSubsystem::setDoneIntaking),
            new WaitCommand(1.5),
            Commands.print("HELICIPTER GOING TO GRAB POS"),
            new ParallelCommandGroup(
                Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(1)),
                helicopterSubsystem.grabPositionCommand())
            // new WaitCommand(1),
            // Commands.print("ELEVATOR GRABBING"),
            // elevatorSubsystem.grabPositionCommand(),
            // Commands.waitUntil(() -> !endEffectorSubsystem.coralLoadedSensor.get()),
            // Commands.print("GOT A CORAL"),
            // Commands.waitSeconds(1),
            // Commands.runOnce(endEffectorSubsystem::stopEndEffector),
            // Commands.print("ELEVATOR GOING TO PRESTOW"),
            // new ParallelCommandGroup(
            //     elevatorSubsystem.preStowPositionCommand(),
            //     endEffectorSubsystem.securePiece()),
            // new WaitCommand(0.2),
            // Commands.print("HELICOPTER STOWING"),
            // helicopterSubsystem.stowPositionCommand(),
            // new WaitCommand(0.4),
            // Commands.runOnce(() -> intakeSubsystem.setHingeDutyCycle(0)),
            // Commands.print("SECURING PIECE"),
            // endEffectorSubsystem.securePiece(),
            // Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(0))
        )/* .onlyIf(() -> !endEffectorSubsystem.getCoralLoaded())*/;
    }

}
