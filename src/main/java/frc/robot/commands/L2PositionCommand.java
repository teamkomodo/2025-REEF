package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class L2PositionCommand extends DynamicCommand {

    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;

    public L2PositionCommand(ElevatorSubsystem elevatorSubsystem, HelicopterSubsystem helicopterSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;

        addRequirements(elevatorSubsystem);
        addRequirements(helicopterSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            new SequentialCommandGroup(
                elevatorSubsystem.clearIntakePositionCommand(),
                Commands.waitUntil(() -> elevatorSubsystem.atCommandedPosition()),
                helicopterSubsystem.l2WaitPositionCommand(),
                Commands.waitUntil(() -> helicopterSubsystem.atCommandedPosition())
            ).onlyIf(() -> (helicopterSubsystem.getPositionWaitingOn() != 2)),
            elevatorSubsystem.l2PositionCommand()
        ).onlyIf(() -> elevatorSubsystem.getZeroed());
    }
}