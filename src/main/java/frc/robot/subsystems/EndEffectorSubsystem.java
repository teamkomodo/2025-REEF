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
    private final DoublePublisher motorVelocityPublisher = endEffectorTable.getDoubleTopic("endEffectorDutyCycle").publish();
    private final DoublePublisher filteredCurrentPublisher = endEffectorTable.getDoubleTopic("filteredCurrent").publish();
    private final BooleanPublisher pieceLoadedSensorPublisher = endEffectorTable.getBooleanTopic("pieceLoadedSensor").publish();
    private final BooleanPublisher coralLoadedPublisher = endEffectorTable.getBooleanTopic("coralLoaded").publish();
    private final BooleanPublisher algaeLoadedPublisher = endEffectorTable.getBooleanTopic("algaeLoaded").publish();

    private final SparkMax endEffectorMotor;
    private final SparkMaxConfig endEffectorMotorConfig;

    private final RelativeEncoder endEffectorEncoder;
    private final SparkClosedLoopController endEffectorController;
    private final PIDGains endEffectorPIDGains = new PIDGains(0.1, 0, 0);

    private final DigitalInput coralLoadedSensor;

    private boolean coralLoaded = false;
    private boolean algaeLoaded = false;
    private double filteredCurrent = 0;
    private double currentFilterConstant = 0.1;

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

    private void checkSensors() {
        coralLoaded = getCoralDetection(coralLoadedSensor);
    }

    private void configMotors() {
        endEffectorMotorConfig
            .inverted(false)
            .smartCurrentLimit(40);

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

    public Command startEndEffectorIntakingCommand() {
        return Commands.runOnce(() -> setEndEffectorDutyCycle(1));
    }
    
    public void stopEndEffector() {
        setEndEffectorDutyCycle(0);
    }

    public boolean getCoralDetection(DigitalInput beamBreak) {
        return !beamBreak.get(); // This is correct!
    }
    
    public boolean getCoralLoaded() {
        return coralLoaded;
    }

    int currentScoreLevel = 0;
    public void setLevel(int level) {
        currentScoreLevel = level;
    }

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

    public Command intakeAlgaeCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setEndEffectorDutyCycle(0.4)),
            Commands.waitUntil(() -> (filteredCurrent > 30)),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> setEndEffectorDutyCycle(0.1)),
            Commands.runOnce(() -> { algaeLoaded = true; })
        );
    }

    public Command ejectCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> {
                if (currentScoreLevel == 1) {
                    setEndEffectorDutyCycle(-0.2);
                } else {
                    setEndEffectorDutyCycle(-0.8);
                }
            }),
            Commands.waitUntil(() -> !getCoralDetection(coralLoadedSensor)),
            Commands.waitSeconds(0.3),
            Commands.runOnce(() -> stopEndEffector()),
            Commands.runOnce(() -> { coralLoaded = false; algaeLoaded = false; })
        );
    }

    public Command scoreAlgaeCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setEndEffectorDutyCycle(-1)),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> stopEndEffector()),
            Commands.runOnce(() -> { coralLoaded = false; algaeLoaded = false; })
        );
    }
    
}