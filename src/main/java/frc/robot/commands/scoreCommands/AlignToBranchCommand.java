package frc.robot.commands.scoreCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.utilityCommands.DynamicCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.scoreCommands.ScoreCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class AlignToBranchCommand extends DynamicCommand{

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
 
    

    public boolean right;
    
   

    
    public AlignToBranchCommand(DrivetrainSubsystem drivetrainSubsystem, EndEffectorSubsystem endEffectorSubsystem, HelicopterSubsystem helicopterSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, Boolean right) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        
        this.right = right;
        
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;


        addRequirements(drivetrainSubsystem);
        addRequirements(endEffectorSubsystem);
        addRequirements(helicopterSubsystem);
        addRequirements(elevatorSubsystem);
        addRequirements(intakeSubsystem);
    }


    @Override
    protected Command getCommand(){
        return new SequentialCommandGroup(
            Commands.run(()-> {drivetrainSubsystem.limelightAlignCommand();}).until(() -> drivetrainSubsystem.atReef),
            Commands.runOnce(() -> {drivetrainSubsystem.goToBranch(right);}),
            Commands.waitSeconds(2),//FIXME: once this works lower the timing to be fest
            ScoreToStowCommand()
        );
    }



    private Command ScoreToStowCommand(){
        return new SequentialCommandGroup(
            helicopterSubsystem.scoreCommand(),
            intakeSubsystem.clearArmPositionCommand(),
            new WaitCommand(0.25),
            endEffectorSubsystem.ejectCommand(),
            new ParallelCommandGroup(helicopterSubsystem.releaseCoralPositionCommand(),
            new WaitCommand(0.1),
            elevatorSubsystem.stowPositionCommand(),
            intakeSubsystem.stowPositionCommand(),
            new WaitCommand(0.2),
            helicopterSubsystem.stowPositionCommand()
        ));
    }
}
