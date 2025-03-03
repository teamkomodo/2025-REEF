package frc.robot.commands.scoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utilityCommands.DynamicCommand;
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
            new StartScoreCommand(helicopterSubsystem, elevatorSubsystem),
            endEffectorSubsystem.ejectCommand(),
            helicopterSubsystem.releaseCoralPositionCommand(),
            Commands.waitSeconds(0.1),
            new SequentialCommandGroup(
                helicopterSubsystem.removeAlgaePositionCommand(),
                Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
                Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(-1))
            ),
            elevatorSubsystem.waitPositionCommand(),
            Commands.waitUntil(elevatorSubsystem::atCommandedPosition),
            Commands.runOnce(() -> endEffectorSubsystem.stopEndEffector()),
            helicopterSubsystem.waitPositionCommand()
        ).onlyIf(() -> (elevatorSubsystem.getZeroed() && helicopterSubsystem.getPositionWaitingOn() >= 3));
    }
}