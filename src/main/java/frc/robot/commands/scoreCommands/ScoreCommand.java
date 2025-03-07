package frc.robot.commands.scoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class ScoreCommand extends DynamicCommand {

    private final EndEffectorSubsystem endEffectorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public ScoreCommand(
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
            Commands.runOnce(() -> System.out.println("Scoring!??!")),
            // Release
            endEffectorSubsystem.ejectCommand(),
            helicopterSubsystem.releaseCoralPositionCommand(),
            // // Return to waiting position
            Commands.waitSeconds(0.1),
            elevatorSubsystem.clearIntakePositionCommand(),
            Commands.waitUntil(elevatorSubsystem::aboveClearIntakePosition),
            helicopterSubsystem.waitPositionCommand(),
            elevatorSubsystem.waitPositionCommand(),
            Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
            Commands.waitUntil(elevatorSubsystem::atCommandedPosition)
        ).onlyIf(() -> (elevatorSubsystem.getZeroed() && helicopterSubsystem.getPositionWaitingOn() != 0));
    }
}