package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.*;

public class IndexerSubsystem extends SubsystemBase {

    // NetworkTable publishers
    private final NetworkTable indexerTable = NetworkTableInstance.getDefault().getTable("indexer");
        // Variable publishers
    private final BooleanPublisher pieceIndexedPublisher = indexerTable.getBooleanTopic("pieceIndexed").publish();
    private final BooleanPublisher pieceInIndexerPublisher = indexerTable.getBooleanTopic("pieceInIndexer").publish();
    private final BooleanPublisher pieceFullyIntakedPublisher = indexerTable.getBooleanTopic("pieceFullyIntaked").publish();
        // Sensor publishers
    private final BooleanPublisher pieceIndexedSensorPublisher = indexerTable.getBooleanTopic("pieceIndexedSensor").publish();
    private final BooleanPublisher pieceInIndexerSensorPublisher = indexerTable.getBooleanTopic("pieceInIndexerSensor").publish();

    // Sensors
    public final DigitalInput coralInIndexerSensor;
    public final DigitalInput coralIndexedSensor;

    // Status variables (and others)
    public boolean coralIndexed = false;
    public boolean coralInIndexer = false;
    public boolean coralFullyInIndexer = false;
    public boolean indexingAllowed = false;

    private boolean coralIndexedAtCurrentCheck;
    private boolean coralInIndexerAtCurrentCheck;
    
    public IndexerSubsystem() {
        // Assign sensors
        coralInIndexerSensor = new DigitalInput(INDEXER_START_SENSOR_CHANNEL); //FIXME: find port number
        coralIndexedSensor = new DigitalInput(INDEXER_END_SENSOR_CHANNEL); //FIXME: find port number

    }

    public void teleopInit() {
        
    }

    @Override
    public void periodic() {
        updateTelemetry();
        checkSensors();
    }

    public void checkSensors() {
        // Set the coral sensed variables
        coralInIndexerAtCurrentCheck = getCoralDetection(coralInIndexerSensor);
        coralIndexedAtCurrentCheck = getCoralDetection(coralIndexedSensor);

        // Handle the coralIndexed sensor
        if (coralIndexedAtCurrentCheck) {
            // Coral is now indexed, set all the variables to true
            coralInIndexer = true;
            coralFullyInIndexer = true;
            coralIndexed = true;
        } else if (coralInIndexerAtCurrentCheck) {
            // The coral is coming into the indexer
            coralInIndexer = true;
        } else if (coralInIndexer && !coralInIndexerAtCurrentCheck) {
            // The coral has gone all the way into the indexer, so record it
            coralFullyInIndexer = true;
        }

        // If coralIndexed and not coralIndexedAtCurrentCheck then the coral got taken so set the variables to false
        if (coralIndexed && !coralIndexedAtCurrentCheck) {
            coralInIndexer = false;
            coralFullyInIndexer = false;
            coralIndexed = false;
        }
    }

    public void updateTelemetry() {
        // Coral status publishing
        pieceIndexedPublisher.set(getPieceIndexed());
        pieceFullyIntakedPublisher.set(getPieceFullyIntaked());
        pieceInIndexerPublisher.set(getPieceInIndexer());
        pieceIndexedSensorPublisher.set(getCoralDetection(coralIndexedSensor));
        pieceInIndexerSensorPublisher.set(getCoralDetection(coralInIndexerSensor));
    }

    public boolean getCoralDetection(DigitalInput beamBreak) {
        return !beamBreak.get();
    }

    public boolean getPieceIndexed() {
        return coralIndexed;
    }

    public boolean getPieceInIndexer() {
        return coralInIndexer;
    }

    public boolean getPieceFullyIntaked() {
        return coralFullyInIndexer;
    }

}
