package frc.robot;

import edu.wpi.first.apriltag.jni.AprilTagJNI.Helper;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.HelicopterSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class StateMachine {
    
    public static enum State {
        START,
        IDLE,
        DRIVE_WITHOUT_PIECE,
        CORAL_INTAKE,
        DRIVE_WITH_PIECE,
        SCORE_CORAL,
        EJECT,
        END
    }

    public static enum IntakeState {
        IDLE,
        START,
        PARTIAL_AQUISITION,
        INDEX,
        PICKUP_INDEXER,
        ALIGN_PIECE,
        STOW
    }

    public static enum ScoreState {
        IDLE,
        START,
        REEF_ALIGN,
        SCORE,
        STOW,
    }

    private static final boolean visionAlign = true;
    private static final boolean automaticScoring = false;
    private static final boolean visionIntake = false;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("teleopstatemachine");
    private final StringPublisher statePublisher = table.getStringTopic("teleopstate").publish();
    private final StringPublisher shootingStatePublisher = table.getStringTopic("shootingstate").publish();
    private final StringPublisher pickupStatePublisher = table.getStringTopic("pickupstate").publish();

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final HelicopterSubsystem helicopterSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final LEDSubsystem ledSubsystem;

    private final XboxController driverController;
    private final XboxController operatorController;

    private State currentState = State.START;
    private IntakeState intakeState = IntakeState.IDLE;
    private ScoreState scoreState = ScoreState.IDLE;

    private boolean stateSwitched = true;
    private boolean shootingStateSwitched = true;
    private boolean currentPickupStateSwitched = true;
    
    private boolean commandingIntake = false;
    private boolean commandingIndex = false;
    private boolean commandingAlignReef = false;
    private boolean commandingScoreReef = false;
    private boolean commandingEject = false;

    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private final Timer timer = new Timer();

    public StateMachine(DrivetrainSubsystem drivetrainSubsystem, 
                        ElevatorSubsystem elevatorSubsystem, 
                        EndEffectorSubsystem endEffectorSubsystem,
                        HelicopterSubsystem helicopterSubsystem,
                        IndexerSubsystem indexerSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        LEDSubsystem ledSubsystem,
                        XboxController driverController,
                        XboxController operatorController) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.helicopterSubsystem = helicopterSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.driverController = driverController;
        this.operatorController = operatorController;
    }

    public void init() {
        currentState = State.START;
        intakeState = IntakeState.IDLE;
        scoreState = ScoreState.IDLE;

        commandingIntake = false;
        commandingIndex = false;
        commandingAlignReef = false;
        commandingScoreReef = false;
        commandingEject = false;
    } 
}
