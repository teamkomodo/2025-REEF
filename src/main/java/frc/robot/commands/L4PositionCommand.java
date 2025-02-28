package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class L4PositionCommand extends DynamicCommand {

    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;

    public L4PositionCommand(ElevatorSubsystem elevatorSubsystem, HelicopterSubsystem helicopterSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;

        addRequirements(elevatorSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(endEffectorSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            elevatorSubsystem.l4PositionCommand(),
            new SequentialCommandGroup(
                Commands.waitUntil(elevatorSubsystem::aboveClearIntakePosition),
                helicopterSubsystem.l4WaitPositionCommand(),
                Commands.waitUntil(helicopterSubsystem::atCommandedPosition)
            ).onlyIf(() -> (helicopterSubsystem.getPositionWaitingOn() != 4))
        ).onlyIf(() -> (elevatorSubsystem.getZeroed() && endEffectorSubsystem.getCoralLoaded()));
    }
}     