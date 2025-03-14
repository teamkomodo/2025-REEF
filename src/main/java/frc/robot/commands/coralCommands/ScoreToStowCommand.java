package frc.robot.commands.coralCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ScoreToStowCommand extends DynamicCommand {

    private final EndEffectorSubsystem endEffectorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LEDSubsystem ledSubsystem;

    public ScoreToStowCommand(
        EndEffectorSubsystem endEffectorSubsystem, 
        HelicopterSubsystem helicopterSubsystem, 
        ElevatorSubsystem elevatorSubsystem,
        IntakeSubsystem intakeSubsystem,
        LEDSubsystem ledSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(endEffectorSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(intakeSubsystem);
        addRequirements(ledSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            intakeSubsystem.stowPositionCommand(),
            new WaitCommand(0.2),
            helicopterSubsystem.scoreCommand(),
            intakeSubsystem.clearArmPositionCommand(),
            new WaitCommand(0.2),
            helicopterSubsystem.releaseCoralPositionCommand(),
            new WaitCommand(0.1),
            endEffectorSubsystem.ejectCommand(),
            intakeSubsystem.stowPositionCommand(),
            //ledSubsystem.flashRedCommand()),
            elevatorSubsystem.stowPositionCommand(),
            new WaitCommand(0.3),
            helicopterSubsystem.stowPositionCommand()
        );
    }
}