package frc.robot.commands.scoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class StartScoreCommand extends DynamicCommand {

    private final HelicopterSubsystem helicopterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public StartScoreCommand(HelicopterSubsystem helicopterSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.helicopterSubsystem = helicopterSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(helicopterSubsystem);
        addRequirements(elevatorSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            // Score and release
            helicopterSubsystem.scoreCommand(),
            Commands.waitUntil(helicopterSubsystem::atCommandedPosition)
        ).onlyIf(elevatorSubsystem::getZeroed);
    }
}