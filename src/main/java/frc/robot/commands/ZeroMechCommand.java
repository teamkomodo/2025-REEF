package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ZeroMechCommand extends DynamicCommand {

    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;

    public ZeroMechCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, HelicopterSubsystem helicopterSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;

        addRequirements(elevatorSubsystem);
        addRequirements(intakeSubsystem);
        addRequirements(helicopterSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            intakeSubsystem.zeroHingeCommand(),
            elevatorSubsystem.zeroElevatorCommand().onlyIf(() -> !elevatorSubsystem.getZeroed())
        ).onlyIf(helicopterSubsystem::isSafeForElevator);
    }
}