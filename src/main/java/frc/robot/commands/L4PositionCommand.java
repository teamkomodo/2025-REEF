package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class L4PositionCommand extends DynamicCommand {

    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;

    public L4PositionCommand(ElevatorSubsystem elevatorSubsystem, HelicopterSubsystem helicopterSubsystem) {
        
        this.elevatorSubsystem = elevatorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;

        addRequirements(elevatorSubsystem);
        addRequirements(helicopterSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            helicopterSubsystem.waitForElevatorPositionCommand(),
            elevatorSubsystem.l4PositionCommand(),
            helicopterSubsystem.l4PositionCommand()
        );
    }
}     