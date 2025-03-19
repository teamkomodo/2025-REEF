package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.PIDGains;

import static frc.robot.Constants.*;

public class EndEffectorSubsystem extends SubsystemBase {
    // NetworkTable publishers
    private final NetworkTable endEffectorTable = NetworkTableInstance.getDefault().getTable("endEffector");
    private final DoublePublisher motorVelocityPublisher = endEffectorTable.getDoubleTopic("endEffectorDutyCycle").publish();
    private final DoublePublisher filteredCurrentPublisher = endEffectorTable.getDoubleTopic("filteredCurrent").publish();
    private final BooleanPublisher pieceLoadedSensorPublisher = endEffectorTable.getBooleanTopic("pieceLoadedSensor").publish();
    private final BooleanPublisher coralLoadedPublisher = endEffectorTable.getBooleanTopic("coralLoaded").publish();
    private final BooleanPublisher algaeLoadedPublisher = endEffectorTable.getBooleanTopic("algaeLoaded").publish();

    private final SparkMax endEffectorMotor;
    private final SparkMaxConfig endEffectorMotorConfig;

    private final RelativeEncoder endEffectorEncoder;
    private final SparkClosedLoopController endEffectorController;
    private final PIDGains endEffectorPIDGains = new PIDGains(1.0, 0, 0);

    public final DigitalInput coralLoadedSensor;

    private boolean coralLoaded = false;
    private boolean algaeLoaded = false;
    private double filteredCurrent = 0;
    private double currentFilterConstant = 0.1;
    private int currentScoreLevel = 0;

    public EndEffectorSubsystem() { // CONSTRUCTION
        endEffectorMotor = new SparkMax(ENDEFFECTOR_MOTOR_ID, SparkMax.MotorType.kBrushless);
        endEffectorMotorConfig = new SparkMaxConfig();

        endEffectorEncoder = endEffectorMotor.getEncoder();
        endEffectorEncoder.setPosition(0);
        endEffectorController = endEffectorMotor.getClosedLoopController();

        // Sensor
        coralLoadedSensor = new DigitalInput(ENDEFFECTOR_SENSOR_CHANNEL);

        configMotors();

    }

    public void teleopInit() {
        setEndEffectorDutyCycle(0); // Stop and freeze the end effector
        holdEndEffector();
    }

    @Override
    public void periodic() {
        filterCurrent();
        checkSensors();
        updateTelemetry();
    }

    private void filterCurrent() {
        filteredCurrent = filteredCurrent * (1 - currentFilterConstant) + endEffectorMotor.getOutputCurrent() * currentFilterConstant;
    }

    private void updateTelemetry() {
        motorVelocityPublisher.set(endEffectorMotor.getOutputCurrent());
        filteredCurrentPublisher.set(filteredCurrent);
        pieceLoadedSensorPublisher.set(getCoralDetection(coralLoadedSensor));
        coralLoadedPublisher.set(coralLoaded);
        algaeLoadedPublisher.set(algaeLoaded);
    }

    // Update the variable with the sensor value
    private void checkSensors() {
        coralLoaded = getCoralDetection(coralLoadedSensor);
    }

    // CONFIGURE THE MOTOR!!
    private void configMotors() {
        endEffectorMotorConfig
            .inverted(false)
            .smartCurrentLimit(40);

        endEffectorMotorConfig.closedLoop
            .pid(endEffectorPIDGains.p, endEffectorPIDGains.i, endEffectorPIDGains.d);

        endEffectorMotor
            .configure(endEffectorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    // Setters
    public void setEndEffectorDutyCycle(double dutyCycle) {
        endEffectorController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

    public void setEndEffectorPosition(double position) {
        endEffectorController.setReference(position, ControlType.kPosition);
    }

    public void runEndEffectorRotations(double rotations) {
        double setPoint = endEffectorEncoder.getPosition() + rotations;
        setEndEffectorPosition(setPoint);
    }

    public void holdEndEffector() {
        setEndEffectorPosition(endEffectorEncoder.getPosition());
    }

    public void stopEndEffector() {
        setEndEffectorDutyCycle(0);
    }

    // Read the sensor
    public boolean getCoralDetection(DigitalInput beamBreak) {
        return !beamBreak.get(); // This is correct!
    }
    
    // Getter
    public boolean getCoralLoaded() {
        return coralLoaded;
    }

    // Tell it where we're scoring so we know how fast to eject
    public void setLevel(int level) {
        currentScoreLevel = level;
    }

    // Grab a coral from the indexer, don't return until we got it!
    public Command intakeCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setEndEffectorDutyCycle(1)),
            Commands.waitUntil(() -> getCoralDetection(coralLoadedSensor)),
            Commands.waitSeconds(0.1),
            Commands.runOnce(() -> setEndEffectorDutyCycle(0.5)),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> setEndEffectorDutyCycle(0.1)),
            Commands.runOnce(() -> { coralLoaded = true; })
        );
    }

    // Start spinning to get the algae out! You better stop it later!!!
    public Command dealgaeifyCommand() {
        return Commands.runOnce(() -> setEndEffectorDutyCycle(-0.9));
    }

    // Score the coral, has the option for a different eject speed for L1 scoring
    public Command ejectCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {
                if (currentScoreLevel == 1) {
                    setEndEffectorDutyCycle(-0.2); // Level 1 score speed
                } else {
                    setEndEffectorDutyCycle(-0.2); // For everything else!
                }
            }),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> stopEndEffector()),
            Commands.runOnce(() -> { coralLoaded = false; algaeLoaded = false; })
        );
    }

    // Get a coral more firmly settled in the end effector
    public Command securePiece() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setEndEffectorDutyCycle(1)),
            new WaitCommand(0.2),
            Commands.runOnce(() -> setEndEffectorDutyCycle(0))
        );
    }

    // Shoot algae out, also will shoot coral out, probably will shoot anything out
    public Command scoreAlgaeCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setEndEffectorDutyCycle(-1)),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> stopEndEffector()),
            Commands.runOnce(() -> { coralLoaded = false; algaeLoaded = false; })
        );
    }
    
}