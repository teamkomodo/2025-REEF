package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.coralCommands.IntakeToStowCommand;
import frc.robot.commands.resetCommands.ResetRobotCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.util.BlinkinPattern;

public class TeleopStateMachine {
    
    // Enum to represent each of the states in our state machine
    public static enum State {
        START,
        DRIVE_WITHOUT_PIECE,
        DRIVE_WITH_PIECE,
        PICKUP_GROUND,
        SCORE_L4,
        SCORE_L3,
        SCORE_L2,
        EJECT_PIECE,
        END
    }

    public static enum ScoringState {
        INACTIVE,
        PREPARE_SCORE,
        READY_SCORE,
        ALIGN_REEF,
        SCORE
    }

    public static enum PickupState {
        INACTIVE,
        NO_PIECE,
        PARTIAL_AQUISITION,
        ALIGN_PIECE
    }

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("teleopstatemachine");
    private final StringPublisher statePublisher = table.getStringTopic("teleopstate").publish();
    private final StringPublisher scoringStatePublisher = table.getStringTopic("scoringstate").publish();
    private final StringPublisher pickupStatePublisher = table.getStringTopic("pickupstate").publish();


    private final BooleanEntry enabledEntry = table.getBooleanTopic("statemachineenabled").getEntry(true);

    // Store a reference to the Command Scheduler so it's easier to schedule commands
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private final Timer timer = new Timer();


    private final DrivetrainSubsystem drivetrainSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LEDSubsystem ledSubsystem;


    private final XboxController driverController;
    private final XboxController operatorController;

    private boolean enabled = true;

    //Store the current state
    public State currentState = State.START;
    private ScoringState currentScoringState = ScoringState.PREPARE_SCORE;
    private PickupState currentPickupState = PickupState.INACTIVE;

    private boolean stateSwitched = true;
    private boolean shootingStateSwitched = true;
    private boolean currentPickupStateSwitched = true;


    private boolean commandingPickupGround = false;
    private boolean commandingEject = false;
    private boolean commandingL2Position = false;
    private boolean commandingL3Position = false;
    private boolean commandingL4Position = false;
    private boolean commandingAlign = false;
    private boolean commandingScore = false;

    public TeleopStateMachine(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, HelicopterSubsystem helicopterSubsystem, EndEffectorSubsystem endEffectorSubsystem, XboxController driverController, XboxController operatorController, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem){
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.ledSubsystem = ledSubsystem;

        this.driverController = driverController;
        this.operatorController = operatorController;

        enabledEntry.set(enabled);
    }


    public void init(){
        currentState = State.START;
        stateSwitched = true;

        currentScoringState = ScoringState.INACTIVE;
        shootingStateSwitched = true;

        currentPickupStateSwitched = true;
        currentPickupState = PickupState.INACTIVE;

        commandingPickupGround = false;
        commandingL2Position = false;
        commandingL3Position = false;
        commandingL4Position = false;
        commandingScore = false;
        commandingEject = false;



    }

    public void periodic(){
        
        if(!enabled)
            return;
        switch (currentState) {
            case START:
                currentState = State.DRIVE_WITHOUT_PIECE;
                stateSwitched = true;
                commandScheduler.schedule(
                    new ResetRobotCommand(intakeSubsystem, elevatorSubsystem, helicopterSubsystem, endEffectorSubsystem, ledSubsystem)
                );
            case DRIVE_WITHOUT_PIECE:

                if(stateSwitched){
                    stateSwitched = false;
                    
                }
                
        }
    }
}
