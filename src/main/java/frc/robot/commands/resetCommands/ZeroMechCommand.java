package frc.robot.commands.resetCommands;

import static frc.robot.Constants.INDEXER_END_SENSOR_CHANNEL;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class ZeroMechCommand extends DynamicCommand {

    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final LEDSubsystem ledSubsystem;

    public ZeroMechCommand(ElevatorSubsystem elevatorSubsystem, 
            IntakeSubsystem intakeSubsystem, 
            HelicopterSubsystem helicopterSubsystem, 
            LEDSubsystem ledSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(elevatorSubsystem);
        addRequirements(intakeSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(ledSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(helicopterSubsystem::holdMotorPosition),
            Commands.runOnce(() -> elevatorSubsystem.setElevatorDutyCycle(0.5)),
            new WaitCommand(.5),
            helicopterSubsystem.stowPositionCommand(),
            Commands.runOnce(() -> elevatorSubsystem.setElevatorDutyCycle(0)),
            intakeSubsystem.zeroHingeCommand(),
            Commands.waitSeconds(0.5),
            elevatorSubsystem.zeroElevatorCommand(),
            ledSubsystem.flashHotPinkCommand()
        ).onlyIf(() -> !elevatorSubsystem.getZeroed() || !intakeSubsystem.getZeroed());
    }
}