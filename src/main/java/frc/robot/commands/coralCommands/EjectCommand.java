package frc.robot.commands.coralCommands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class EjectCommand extends DynamicCommand {
    //test
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public EjectCommand(
            IntakeSubsystem intakeSubsystem, 
            EndEffectorSubsystem endEffectorSubsystem,
            LEDSubsystem ledSubsystem,
            HelicopterSubsystem helicopterSubsystem,
            ElevatorSubsystem elevatorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(intakeSubsystem);
        addRequirements(endEffectorSubsystem);
        addRequirements(ledSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(elevatorSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            elevatorSubsystem.l3PositionCommand(),
            intakeSubsystem.intakePositionCommand(),
            new WaitCommand(0.25),
            helicopterSubsystem.l1WaitPositionCommand(),
            new WaitCommand(0.25),
            elevatorSubsystem.l1PositionCommand(),
            Commands.runOnce(() -> intakeSubsystem.setIntakeDutyCycle(-1.0)),
            Commands.runOnce(() -> endEffectorSubsystem.setEndEffectorDutyCycle(-1.0)),
            Commands.waitSeconds(0.3),
            new ParallelCommandGroup(Commands.runOnce(intakeSubsystem::stopIntake),
            Commands.runOnce(endEffectorSubsystem::stopEndEffector),
            intakeSubsystem.stowPositionCommand(),
            elevatorSubsystem.stowPositionCommand(),
            helicopterSubsystem.stowPositionCommand())
        );
    }
}