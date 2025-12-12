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

public class FinishIntakeCommand extends DynamicCommand{
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final LEDSubsystem ledSubsystem;
   

    public FinishIntakeCommand(
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
            Commands.print("ELEVATOR GRABBING"),
            Commands.runOnce(() -> intakeSubsystem.stopIntake(), intakeSubsystem),
            intakeSubsystem.intakePositionCommand(),
            elevatorSubsystem.grabPositionCommand(),
            helicopterSubsystem.grabPositionCommand(),
            endEffectorSubsystem.intakeCommand(),
            //Commands.waitUntil(() -> !endEffectorSubsystem.coralLoadedSensor.get()),
            Commands.waitSeconds(1),
            Commands.print("GOT A CORAL"),
            endEffectorSubsystem.securePiece(),
            Commands.runOnce(() -> { endEffectorSubsystem.coralLoaded = true; }),
            Commands.print("ELEVATOR GOING TO PRESTOW"),
            new ParallelCommandGroup(
                elevatorSubsystem.preStowPositionCommand(),
                endEffectorSubsystem.securePiece()),
            new WaitCommand(0.5),
            Commands.print("HELICOPTER STOWING"),
            helicopterSubsystem.stowPositionCommand(),
            new WaitCommand(0.4),
            Commands.runOnce(() -> intakeSubsystem.setHingeDutyCycle(0)),
            Commands.print("SECURING PIECE"),
            endEffectorSubsystem.securePiece(),
            Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(0)),
            elevatorSubsystem.stowPositionCommand(),
            Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(0.1), endEffectorSubsystem),
           // intakeSubsystem.stowPositionCommand(),
            //Commands.waitUntil(() -> endEffectorSubsystem.getCoralLoaded()),
            Commands.print("FINISHED")
            
        ).onlyIf(() -> !endEffectorSubsystem.getCoralLoaded());
    }
}
