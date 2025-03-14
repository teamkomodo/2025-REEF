package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.EndEffectorSubsystem;

public class EjectCommand extends DynamicCommand {
    //test
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final LEDSubsystem ledSubsystem;

    public EjectCommand(
            IntakeSubsystem intakeSubsystem, 
            EndEffectorSubsystem endEffectorSubsystem,
            LEDSubsystem ledSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(intakeSubsystem);
        addRequirements(endEffectorSubsystem);
        addRequirements(ledSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> intakeSubsystem.startIntake()),
            Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(-0.7)),
            Commands.waitSeconds(0.6),
            new ParallelCommandGroup(Commands.runOnce(intakeSubsystem::stopIntake),
            Commands.runOnce(endEffectorSubsystem::stopEndEffector),
            ledSubsystem.flashRedCommand())
        );
    }
}