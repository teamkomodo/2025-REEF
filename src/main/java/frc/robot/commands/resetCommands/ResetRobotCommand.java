package frc.robot.commands.resetCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class ResetRobotCommand extends DynamicCommand {

    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final LEDSubsystem ledSubsystem;

    public ResetRobotCommand(
            IntakeSubsystem intakeSubsystem, 
            ElevatorSubsystem elevatorSubsystem, 
            HelicopterSubsystem helicopterSubsystem, 
            EndEffectorSubsystem endEffectorSubsystem,
            LEDSubsystem ledSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(intakeSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(endEffectorSubsystem);
        addRequirements(ledSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
          Commands.runOnce(intakeSubsystem::stopIntake),
          Commands.runOnce(endEffectorSubsystem::stopEndEffector),
          new SequentialCommandGroup(
            elevatorSubsystem.clearIndexerPositionCommand(),
            new WaitCommand(0.6)
          ).onlyIf(() -> !helicopterSubsystem.isSafeForElevator()),
          helicopterSubsystem.stowPositionCommand(),
          Commands.waitUntil(() -> helicopterSubsystem.atCommandedPosition()),
          intakeSubsystem.stowPositionCommand(),
          elevatorSubsystem.zeroElevatorCommand(),
          elevatorSubsystem.minPositionCommand()
        ).onlyIf(() -> elevatorSubsystem.getZeroed() && intakeSubsystem.getZeroed());
    }
}