package frc.robot.commands.utilityCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EjectCommand extends DynamicCommand {
    //test
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;

    public EjectCommand(
            IntakeSubsystem intakeSubsystem, 
            EndEffectorSubsystem endEffectorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;

        addRequirements(intakeSubsystem);
        addRequirements(endEffectorSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> intakeSubsystem.startIntake()),
            Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(-0.7)),
            Commands.waitSeconds(0.6),
            Commands.runOnce(intakeSubsystem::stopIntake),
            Commands.runOnce(endEffectorSubsystem::stopEndEffector)
        );
    }
}