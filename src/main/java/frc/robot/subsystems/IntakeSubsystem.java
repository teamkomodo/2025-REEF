package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.PIDGains;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {

    // NetworkTable publishers
    private final NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("intake");
        // Motor publishers
    private final DoublePublisher hingeMotorPositionPublisher = intakeTable.getDoubleTopic("hingeMotorPosition").publish();
    private final DoublePublisher hingeMotorSupposedPositionPublisher = intakeTable.getDoubleTopic("hingeMotorSupposedPosition").publish();
    private final DoublePublisher hingeMotorDutyCyclePublisher = intakeTable.getDoubleTopic("motorDutyCycle").publish();
    private final DoublePublisher intakeVelocityPublisher = intakeTable.getDoubleTopic("intakeDutyCycle").publish();
        // Variable publishers
    private final BooleanPublisher pieceIntakedPublisher = intakeTable.getBooleanTopic("pieceInIntake").publish();
        // Sensor publishers
    private final BooleanPublisher pieceIntakedSensorPublisher = intakeTable.getBooleanTopic("pieceIntakedSensor").publish();
    private final BooleanPublisher pieceIntakedSensor2Publisher = intakeTable.getBooleanTopic("pieceIntakedSensor2").publish();
    private final BooleanPublisher atLimitSwitchPublisher = intakeTable.getBooleanTopic("atLimitSwitch").publish();
    private final BooleanPublisher zeroedPublisher = intakeTable.getBooleanTopic("zeroed").publish();


    // Intake motors
    private final SparkMax intakeMotor;
    private final SparkMaxConfig intakeMotorConfig;

    private final RelativeEncoder intakeEncoder;
    private final SparkClosedLoopController intakeController;

    // Hinge motors
    private final SparkMax hingeMotor;
    private final SparkMaxConfig hingeMotorConfig;
    private final SoftLimitConfig hingeSoftLimitConfig;

    private final RelativeEncoder hingeEncoder;
    private final SparkClosedLoopController hingeController;

    // PID constants
    private final PIDGains intakePIDGains = new PIDGains(1, 0, 0);
    private final PIDGains hingePIDGains = new PIDGains(1, 0, 0);
    private final double hingeMaxAccel = 1000;
    private final double hingeMaxVelocity = 1000;
    private final double hingeAllowedClosedLoopError = 1;

    // Sensors
    public final DigitalInput coralIntakedSensor;
    public final DigitalInput coralIntakedSensor2;
    private final DigitalInput hingeLimitSwitch;

    // Status variables (and others)
    public String currentCommand = "idle";

    private double filteredCurrent = 0;
    private double currentFilterConstant = 0.1;

    private boolean coralInIntake = false;

    private boolean limitSwitchAtCurrentCheck;
    private boolean limitSwitchAtLastCheck;
    private boolean zeroed = false;

    private double hingeSupposedPosition = 0;
    
    public IntakeSubsystem() {
        // Assign intake motors
        intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeMotorConfig = new SparkMaxConfig();

        // Assign the first hinge motor
        hingeMotor = new SparkMax(INTAKE_HINGE_MOTOR_ID, MotorType.kBrushless);
        hingeMotorConfig = new SparkMaxConfig();
        hingeSoftLimitConfig = new SoftLimitConfig();

        // Assign sensors
        coralIntakedSensor = new DigitalInput(CORAL_INTAKE_SENSOR_CHANNEL);
        coralIntakedSensor2 = new DigitalInput(CORAL_INTAKE_SENSOR_2_CHANNEL);
        hingeLimitSwitch = new DigitalInput(INTAKE_HINGE_ZERO_SWITCH_CHANNEL);

        // Assign intake encoder and controller
        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setPosition(0);
        intakeController = intakeMotor.getClosedLoopController();

        // Assign hinge encoder and controller
        hingeEncoder = hingeMotor.getEncoder();
        hingeEncoder.setPosition(0);
        hingeController = hingeMotor.getClosedLoopController();

        configMotors();
        setIntakeDutyCycle(0);
        setHingeDutyCycle(0);
    }

    public void teleopInit() {
        setIntakeDutyCycle(0);
        setHingeDutyCycle(0);
    }

    @Override
    public void periodic() {
        updateTelemetry();
        filterCurrent();
        checkSensors();
        checkLimitSwitch();
    }

    private void filterCurrent() {
        filteredCurrent = filteredCurrent * (1 - currentFilterConstant) + intakeMotor.getOutputCurrent() * currentFilterConstant;
    }

    public void checkSensors() {
        if (getCoralDetection(coralIntakedSensor) || getCoralDetection(coralIntakedSensor2)) {
            coralInIntake = true;
        }
    }

    public void updateTelemetry() {
        // Coral status publishing
        pieceIntakedPublisher.set(coralInIntake);
        pieceIntakedSensorPublisher.set(getCoralDetection(coralIntakedSensor));
        pieceIntakedSensor2Publisher.set(getCoralDetection(coralIntakedSensor2));
        // Intake publishing
        intakeVelocityPublisher.set(intakeMotor.getOutputCurrent());
        // Hinge publishing
        hingeMotorPositionPublisher.set(hingeEncoder.getPosition());
        hingeMotorSupposedPositionPublisher.set(hingeSupposedPosition);
        hingeMotorDutyCyclePublisher.set(hingeEncoder.getVelocity());
        atLimitSwitchPublisher.set(getLimitSwitchAtCurrentCheck());
        zeroedPublisher.set(zeroed);
    }

    public void setIntakeDutyCycle(double dutyCycle) {
        intakeController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

    private void setHingeMinMaxLimit() {
        hingeSoftLimitConfig
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        hingeMotorConfig.apply(hingeSoftLimitConfig);

        hingeMotor.configure(hingeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configMotors() {
        // Intake motors
        intakeMotorConfig
            .inverted(false)
            .smartCurrentLimit(80);

        intakeMotorConfig.closedLoop
            .pid(intakePIDGains.p, intakePIDGains.i, intakePIDGains.d);
        
        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Hinge motors
        hingeSoftLimitConfig
            .forwardSoftLimit(INTAKE_HINGE_MAX_POSITION)
            .reverseSoftLimit(INTAKE_HINGE_MIN_POSITION)
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);

        hingeMotorConfig
            .inverted(false)
            .smartCurrentLimit(80)
            .apply(hingeSoftLimitConfig);

        hingeMotorConfig.closedLoop
            .pid(hingePIDGains.p, hingePIDGains.i, hingePIDGains.d)
            .maxMotion.maxAcceleration(hingeMaxAccel)
            .maxVelocity(hingeMaxVelocity)
            .allowedClosedLoopError(hingeAllowedClosedLoopError);

        hingeMotor.configure(hingeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void checkLimitSwitch() {
        limitSwitchAtCurrentCheck = getLimitSwitchAtCurrentCheck();
        if (limitSwitchAtCurrentCheck && !limitSwitchAtLastCheck) {
            hingeEncoder.setPosition(INTAKE_HINGE_MIN_POSITION);
            if (!zeroed) {
                setHingePosition(INTAKE_HINGE_MIN_POSITION);
                setHingeMinMaxLimit();
            }
            zeroed = true;
        }
        limitSwitchAtLastCheck = limitSwitchAtCurrentCheck;
    }

    public boolean getCoralDetection(DigitalInput beamBreak) {
        return !beamBreak.get();
    }

    public double getIntakeVelocity() {
        return intakeEncoder.getVelocity();
    }

    public Command ejectCommand() {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setIntakeDutyCycle(-0.5)),
            Commands.waitSeconds(0.6),
            Commands.runOnce(() -> stopIntake())
        );
    }

    public void startIntake() {
        setIntakeDutyCycle(INTAKE_SPEED);
    }

    public void stopIntake() {
        setIntakeDutyCycle(0);
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
    }

    public void setHingePosition(double position) {
        hingeSupposedPosition = position;
        hingeController.setReference(position, ControlType.kMAXMotionPositionControl);
    }
    
    public void setHingeDutyCycle(double dutyCycle) {
        hingeController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

    public void holdHingePosition() {
        setHingePosition(hingeEncoder.getPosition());
    }

    public Command stowPositionCommand() {
        return this.runOnce(() -> setHingePosition(INTAKE_HINGE_STOW_POSITION));
    }

    public Command clearCoralPositionCommand() {
        return this.runOnce(() -> setHingePosition(INTAKE_HINGE_CLEAR_CORAL_POSITION));
    }

    public Command stationIntakePositionCommand() {
        return this.runOnce(() -> setHingePosition(INTAKE_HINGE_STATION_INTAKE_POSITION));
    }

    public Command intakePositionCommand() {
        return this.runOnce(() -> setHingePosition(INTAKE_HINGE_INTAKE_POSITION));
    }

    public Command zeroHingeCommand() { // Code uses this function
        // Activate with one press
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setHingeDutyCycle(INTAKE_HINGE_ZEROING_SPEED)),
            Commands.waitUntil(() -> (zeroed)),
            Commands.runOnce(() -> setHingeDutyCycle(0))//,
            // stowPositionCommand()
        ).onlyIf(() -> (!zeroed));
    }

    public boolean getLimitSwitchAtCurrentCheck() {
        return !hingeLimitSwitch.get();
    }

    public void setDoneIntaking() {
        coralInIntake = false;
    }

    public boolean getPieceInIntake() {
        return coralInIntake;
    }

    public boolean atCommandedPosition() {
        return Math.abs(hingeEncoder.getPosition() - hingeSupposedPosition) < hingeAllowedClosedLoopError;
    }

    public boolean isSafeForElevator() {
        return hingeEncoder.getPosition() > INTAKE_HINGE_SAFE_ELEVATOR_POSITION;
    }

    public boolean getZeroed() {
        return zeroed;
    }
}
