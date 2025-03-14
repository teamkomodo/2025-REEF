package frc.robot.commands.utilityCommands;

import static frc.robot.Constants.INDEXER_END_SENSOR_CHANNEL;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            Commands.runOnce(helicopterSubsystem::holdMotorPosition),
            new SequentialCommandGroup(
              Commands.runOnce(() -> elevatorSubsystem.setElevatorDutyCycle(0.5)),
              new WaitCommand(0.3),
              Commands.runOnce(() -> elevatorSubsystem.setElevatorDutyCycle(0))
            ),
            helicopterSubsystem.stowPositionCommand(),
            Commands.waitSeconds(0.5),
            intakeSubsystem.zeroHingeCommand(),
            elevatorSubsystem.zeroElevatorCommand()
        ).onlyIf(() -> !elevatorSubsystem.getZeroed() || !intakeSubsystem.getZeroed());
    }
}