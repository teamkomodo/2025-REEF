package frc.robot.commands.algaeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class KnockOutLowAlgaeCommand extends DynamicCommand {

    private final EndEffectorSubsystem endEffectorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public KnockOutLowAlgaeCommand(
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
            elevatorSubsystem.clearIndexerPositionCommand(),
            Commands.waitUntil(elevatorSubsystem::aboveCommandedPosition),
            helicopterSubsystem.highAlgaePositionCommand(),
            Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
            elevatorSubsystem.highAlgaePositionCommand(),

            endEffectorSubsystem.dealgaeifyCommand(),

            helicopterSubsystem.stowPositionCommand(),
            Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
            elevatorSubsystem.stowPositionCommand()
        ).onlyIf(elevatorSubsystem::getZeroed);
    }
}