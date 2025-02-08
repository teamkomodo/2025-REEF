package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class ScoreCommand extends DynamicCommand {

    private final EndEffectorSubsystem endEffectorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;

    public ScoreCommand(EndEffectorSubsystem endEffectorSubsystem, HelicopterSubsystem helicopterSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;

        addRequirements(endEffectorSubsystem);
        addRequirements(helicopterSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            helicopterSubsystem.scoreCommand(),
            endEffectorSubsystem.ejectCommand(),
            helicopterSubsystem.releaseCoralCommand()
        );
    }
}