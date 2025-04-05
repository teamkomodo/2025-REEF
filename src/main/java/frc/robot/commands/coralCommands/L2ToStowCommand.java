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

public class L2ToStowCommand extends DynamicCommand {

    private final EndEffectorSubsystem endEffectorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LEDSubsystem ledSubsystem;

    public L2ToStowCommand(
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
            new WaitCommand(0.2),
            helicopterSubsystem.releaseCoralPositionCommand(),
            //helicopterSubsystem.scoreCommand(),
            new WaitCommand(0.2),
            endEffectorSubsystem.ejectCommand(),
            new WaitCommand(0.1),
            intakeSubsystem.intakePositionCommand(),
            new WaitCommand(0.45),
            Commands.runOnce(() -> intakeSubsystem.setHingeDutyCycle(0))
            );
    }
}