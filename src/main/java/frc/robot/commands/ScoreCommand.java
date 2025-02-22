package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
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
            // Release
            endEffectorSubsystem.ejectCommand(),
            helicopterSubsystem.releaseCoralPositionCommand(),
            // // Return to waiting position
            Commands.waitSeconds(0.3),
            elevatorSubsystem.clearIntakePositionCommand(),
            Commands.waitUntil(() -> elevatorSubsystem.atCommandedPosition()),
            helicopterSubsystem.grabPositionCommand(),
            Commands.waitUntil(() -> helicopterSubsystem.atCommandedPosition()),
            elevatorSubsystem.waitPositionCommand()
        ).onlyIf(() -> (elevatorSubsystem.getZeroed()));
    }
}