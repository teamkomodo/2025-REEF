package frc.robot.commands.scoreCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class CompleteScoreCommand extends DynamicCommand {

    private final EndEffectorSubsystem endEffectorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public CompleteScoreCommand(
                EndEffectorSubsystem endEffectorSubsystem, HelicopterSubsystem helicopterSubsystem, 
                ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(endEffectorSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(intakeSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            intakeSubsystem.clearArmPositionCommand(),
            Commands.runOnce(() -> endEffectorSubsystem.setLevel(helicopterSubsystem.getPositionWaitingOn())),
            
            // Score and release
            helicopterSubsystem.scoreCommand(),
            new WaitCommand(0.25),
            //Commands.waitUntil(helicopterSubsystem::atCommandedPosition),
            
            // Release
            endEffectorSubsystem.ejectCommand(),
            helicopterSubsystem.releaseCoralPositionCommand(),
            // // Return to waiting position
            Commands.waitSeconds(0.1),
            helicopterSubsystem.waitPositionCommand(),
            elevatorSubsystem.waitPositionCommand()
        ).onlyIf(() -> (elevatorSubsystem.getZeroed() && helicopterSubsystem.getPositionWaitingOn() != 0));
    }
}