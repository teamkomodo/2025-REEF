package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import static frc.robot.Constants.*;

public class IndexerSubsystem extends SubsystemBase {

    // NetworkTable publishers
    private final NetworkTable indexerTable = NetworkTableInstance.getDefault().getTable("indexer");
        // Motor publishers
    private final DoublePublisher indexerVelocityPublisher = indexerTable.getDoubleTopic("indexerBeltLeftDutyCycle").publish();
        // Variable publishers
    private final BooleanPublisher pieceIndexedPublisher = indexerTable.getBooleanTopic("pieceIndexed").publish();
    private final BooleanPublisher pieceInIndexerPublisher = indexerTable.getBooleanTopic("pieceInIndexer").publish();
    private final StringPublisher currentStatePublisher = indexerTable.getStringTopic("currentState").publish();
        // Sensor publishers
    private final BooleanPublisher pieceIndexedSensorPublisher = indexerTable.getBooleanTopic("pieceIndexedSensor").publish();
    private final BooleanPublisher pieceInIndexerSensorPublisher = indexerTable.getBooleanTopic("pieceInIndexerSensor").publish();

    // Left belt motor
    private final SparkMax leftBeltMotor;
    private final SparkMaxConfig leftBeltMotorConfig;
    private final RelativeEncoder leftBeltEncoder;
    private final SparkClosedLoopController leftBeltController;

    // Right belt motor
    private final SparkMax rightBeltMotor;
    private final SparkMaxConfig rightBeltMotorConfig;
    private final RelativeEncoder rightBeltEncoder;
    private final SparkClosedLoopController rightBeltController;

    // Centering motor
    private final SparkMax centeringMotor;
    private final SparkMaxConfig centeringMotorConfig;
    private final RelativeEncoder centeringEncoder;
    private final SparkClosedLoopController centeringController;

    // Sensors
    public final DigitalInput coralInIndexerSensor;
    public final DigitalInput coralIndexedSensor;

    // Status variables (and others)
    public String currentState = "idle";

    public boolean coralIndexed = false;
    public boolean coralInIndexer = false;

    private double filteredCurrent = 0;
    private double currentFilterConstant = 0.1;

    private boolean coralIndexedAtCurrentCheck;
    private boolean coralInIndexerAtCurrentCheck;
    private boolean coralIndexedAtLastCheck;
    private boolean coralInIndexerAtLastCheck;
    
    public IndexerSubsystem() {
        // Assign left belt motor, encoder, and controller
        leftBeltMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless); //FIXME: find motor id
        leftBeltMotorConfig = new SparkMaxConfig();
        leftBeltEncoder = leftBeltMotor.getEncoder();
        leftBeltEncoder.setPosition(0);
        leftBeltController = leftBeltMotor.getClosedLoopController();

        // Assign right belt motor, encoder, and controller
        rightBeltMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless); //FIXME: find motor id
        rightBeltMotorConfig = new SparkMaxConfig();
        rightBeltEncoder = rightBeltMotor.getEncoder();
        rightBeltEncoder.setPosition(0);
        rightBeltController = rightBeltMotor.getClosedLoopController();

        // Assign centering motor, encoder, and controller
        centeringMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless); //FIXME: find motor id
        centeringMotorConfig = new SparkMaxConfig();
        centeringEncoder = centeringMotor.getEncoder();
        centeringEncoder.setPosition(0);
        centeringController = centeringMotor.getClosedLoopController();

        // Assign sensors
        coralInIndexerSensor = new DigitalInput(INDEXER_START_SENSOR_CHANNEL); //FIXME: find port number
        coralIndexedSensor = new DigitalInput(INDEXER_END_SENSOR_CHANNEL); //FIXME: find port number

        configMotors();
        stopIndexer();
    }

    public void teleopInit() {
        stopIndexer();
    }

    @Override
    public void periodic() {
        updateTelemetry();
        filterCurrent();
        checkSensors();
    }

    private void filterCurrent() {
        filteredCurrent = filteredCurrent * (1 - currentFilterConstant) + leftBeltMotor.getOutputCurrent() * currentFilterConstant;
    }

    public void checkSensors() {
        // Set the coral sensed variables
        coralInIndexerAtCurrentCheck = getCoralDetection(coralInIndexerSensor);
        coralIndexedAtCurrentCheck = getCoralDetection(coralIndexedSensor);
        
        // Handle the coralIndexed sensor
        if (coralIndexedAtCurrentCheck) {
            coralInIndexer = true;
            coralIndexed = true;
            stopIndexer();
        } else if (coralInIndexerAtCurrentCheck) {
            coralInIndexer = true;
            startIndexer();
        }
        // If coralIndexed and not coralIndexedAtLastCheck then the coral got taken so set coralInIndexer and coralIndexed to false
        if (coralIndexed && !coralIndexedAtLastCheck) {
            coralInIndexer = false;
            coralIndexed = false;
        }

        // Set the last check variables to the current check variables
        coralInIndexerAtLastCheck = coralInIndexerAtCurrentCheck;
        coralIndexedAtLastCheck = coralIndexedAtCurrentCheck;
    }

    public void updateTelemetry() {
        // Coral status publishing
        pieceIndexedPublisher.set(getPieceIndexed());
        pieceInIndexerPublisher.set(getPieceInIndexer());
        pieceIndexedSensorPublisher.set(coralInIndexerSensor.get());
        pieceInIndexerSensorPublisher.set(coralIndexedSensor.get());
        currentStatePublisher.set(currentState);
        // Indexer publishing
        indexerVelocityPublisher.set(leftBeltMotor.getOutputCurrent());
    }

    private void configMotors() {
        // Left belt motor
        leftBeltMotorConfig
            .inverted(false)
            .smartCurrentLimit(80);

        leftBeltMotorConfig.closedLoop
            .pid(1, 0, 0);

        leftBeltMotor.configure(leftBeltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Right belt motor
        rightBeltMotorConfig
            .inverted(false)
            .smartCurrentLimit(80);
        
        rightBeltMotorConfig.closedLoop
            .pid(1, 0, 0);
        
        rightBeltMotor.configure(rightBeltMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Centering motor
        centeringMotorConfig
            .inverted(false)
            .smartCurrentLimit(80);
        
        centeringMotorConfig.closedLoop
            .pid(1, 0, 0);
        
        centeringMotor.configure(centeringMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setIndexerDutyCycle(double dutyCycle) {
        leftBeltController.setReference(dutyCycle, ControlType.kDutyCycle);
        rightBeltController.setReference(dutyCycle * 0.8, ControlType.kDutyCycle);
        centeringController.setReference(dutyCycle * 0.7, ControlType.kDutyCycle);
    }

    public boolean getCoralDetection(DigitalInput beamBreak) {
        return beamBreak.get();
    }

    public boolean getPieceIndexed() {
        return coralIndexedAtCurrentCheck;
    }

    public boolean getPieceInIndexer() {
        return coralInIndexerAtCurrentCheck;
    }

    public double getIndexerVelocity() {
        return leftBeltEncoder.getVelocity();
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
    }

    public void stopIndexer() {
        currentState = "idle";
        setIndexerDutyCycle(0);
    }

    public void startIndexer() {
        currentState = "running";
        setIndexerDutyCycle(0.5);
    }

}
