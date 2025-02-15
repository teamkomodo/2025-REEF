package frc.robot.commands;

import java.lang.constant.DirectMethodHandleDesc;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;

public class ScoreCommand extends DynamicCommand {

    private final EndEffectorSubsystem endEffectorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final DrivetrainSubsystem drivetrainSubsystem;

    public ScoreCommand(
                EndEffectorSubsystem endEffectorSubsystem, HelicopterSubsystem helicopterSubsystem, 
                ElevatorSubsystem elevatorSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;

        addRequirements(endEffectorSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    protected Command getCommand() {
        return new SequentialCommandGroup(
            // Score and release
            helicopterSubsystem.scoreCommand(),
            endEffectorSubsystem.ejectCommand(),
            helicopterSubsystem.releaseCoralPositionCommand(),
            drivetrainSubsystem.backOffCommand(),
            // Return to waiting position
            elevatorSubsystem.clearIntakePositionCommand(),
            helicopterSubsystem.grabPositionCommand(),
            elevatorSubsystem.waitPositionCommand()
        ).onlyIf(() -> (elevatorSubsystem.getZeroed()));
    }
}