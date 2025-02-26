package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class ScoreAndRemoveAlgaeCommand extends DynamicCommand {

    private final EndEffectorSubsystem endEffectorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public ScoreAndRemoveAlgaeCommand(
            EndEffectorSubsystem endEffectorSubsystem, HelicopterSubsystem helicopterSubsystem, 
            ElevatorSubsystem elevatorSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(endEffectorSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(elevatorSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            endEffectorSubsystem.ejectCommand(),
            helicopterSubsystem.releaseCoralPositionCommand(),

            new SequentialCommandGroup(
                helicopterSubsystem.removeAlgaePositionCommand(),
                endEffectorSubsystem.removeAlgaeCommand(),
                Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
                endEffectorSubsystem.removeAlgaeCommand()
            ).onlyIf(() -> (helicopterSubsystem.getPositionWaitingOn() >= 3)),
            
            elevatorSubsystem.clearIntakePositionCommand(),
            helicopterSubsystem.grabPositionCommand(),
            elevatorSubsystem.waitPositionCommand()
        ).onlyIf(elevatorSubsystem::getZeroed);
    }
}