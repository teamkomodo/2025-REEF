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
import frc.robot.util.PIDGains;

import static frc.robot.Constants.*;

public class EndEffectorSubsystem extends SubsystemBase {
    // NetworkTable publishers
    private final NetworkTable endEffectorTable = NetworkTableInstance.getDefault().getTable("endEffector");
    private final DoublePublisher motorVelocityPublisher = endEffectorTable.getDoubleTopic("intakeDutyCycle").publish();
    private final BooleanPublisher pieceLoadedSensorPublisher = endEffectorTable.getBooleanTopic("pieceLoadedSensor").publish();
    private final BooleanPublisher pieceLoadedPublisher = endEffectorTable.getBooleanTopic("pieceLoaded").publish();


    private final SparkMax endEffectorMotor;
    private final SparkMaxConfig endEffectorMotorConfig;

    private final RelativeEncoder endEffectorEncoder;
    private final SparkClosedLoopController endEffectorController;
    private final PIDGains endEffectorPIDGains = new PIDGains(0.1, 0, 0);

    private final DigitalInput coralLoadedSensor;

    private boolean coralLoaded = false;


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

    @Override
    public void periodic() {
        checkSensors();
        updateTelemetry();
    }

    private void updateTelemetry() {
        motorVelocityPublisher.set(endEffectorMotor.getOutputCurrent());
        pieceLoadedSensorPublisher.set(coralLoadedSensor.get());
        pieceLoadedPublisher.set(coralLoaded);
    }

    private void checkSensors() {
        coralLoaded = getCoralDetection(coralLoadedSensor);
    }

    private void configMotors() {
        endEffectorMotorConfig
            .inverted(false)
            .smartCurrentLimit(50); //FIXME: set current limit

        endEffectorMotorConfig.closedLoop
            .pid(endEffectorPIDGains.p, endEffectorPIDGains.i, endEffectorPIDGains.d);

        endEffectorMotor
            .configure(endEffectorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

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

    public boolean getCoralDetection(DigitalInput beamBreak) {
        return !beamBreak.get(); // This is correct!
    }
    
    public boolean getCoralLoaded() {
        return coralLoaded;
    }

    public Command intakeCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setEndEffectorDutyCycle(0.9)),
            Commands.waitUntil(() -> getCoralDetection(coralLoadedSensor)),
            Commands.runOnce(() -> setEndEffectorDutyCycle(0.3)),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> setEndEffectorDutyCycle(0.05)),
            Commands.runOnce(() -> { coralLoaded = true; })
        );
    }

    public Command ejectCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setEndEffectorDutyCycle(-1)),
            Commands.waitUntil(() -> !getCoralDetection(coralLoadedSensor)),
            Commands.waitSeconds(0.7),
            Commands.runOnce(() -> setEndEffectorDutyCycle(0)),
            Commands.runOnce(() -> { coralLoaded = false; })
        );
    }

    public Command removeAlgaeCommand() {
        return Commands.runOnce(() -> runEndEffectorRotations(4));
    }
    
}