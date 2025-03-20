package frc.robot.commands;

import static frc.robot.Constants.INTAKE_HINGE_FEED_CORAL_POSITION;
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
import frc.robot.subsystems.LEDSubsystem;



public class AlgaePickup extends DynamicCommand{
    
    private final IntakeSubsystem intakeSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final LEDSubsystem ledSubsystem;
   

    public AlgaePickup(
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
            Commands.runOnce(() -> elevatorSubsystem.setElevatorSupposedPosition(3)),
            Commands.runOnce(() -> helicopterSubsystem.setMotorPosition(0.88)),
            Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(1)),
            Commands.waitUntil(() -> !endEffectorSubsystem.coralLoadedSensor.get()),
            Commands.runOnce(() -> intakeSubsystem.setIntakeDutyCycle(0)),
            intakeSubsystem.stowPositionCommand()
        );
    }
}