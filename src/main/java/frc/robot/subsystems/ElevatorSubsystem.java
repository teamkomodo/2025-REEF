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

public class ElevatorSubsystem extends SubsystemBase {

    // NetworkTable publishers
    private final NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable("elevator");
        // Motor publishers
    private final DoublePublisher elevatorMotorPositionPublisher = elevatorTable.getDoubleTopic("motorPosition").publish();
    private final DoublePublisher elevatorMotorSupposedPositionPublisher = elevatorTable.getDoubleTopic("motorSupposedPosition").publish();
    private final DoublePublisher elevatorMotorDutyCyclePublisher = elevatorTable.getDoubleTopic("motorDutyCycle").publish();
        // Sensor publisher
    private final BooleanPublisher atLimitSwitchPublisher = elevatorTable.getBooleanTopic("atLimitSwitch").publish();
    private final BooleanPublisher zeroedPublisher = elevatorTable.getBooleanTopic("zeroed").publish();

    // Intake motors
    private final SparkMax elevatorMotor;
    private final SparkMaxConfig elevatorMotorConfig;
    private final RelativeEncoder elevatorEncoder;
    private final SparkClosedLoopController elevatorController;
    private final SoftLimitConfig softLimitConfig;

    // PID constants
    private final PIDGains elevatorPIDGains = new PIDGains(0.2, 0.00001, 0.002, 0.00);
    private final double elevatorMaxAccel = 3000;
    private final double elevatorMaxVelocity = 3000;
    private final double elevatorAllowedClosedLoopError = 0.2;

    // Sensors
    private final DigitalInput limitSwitch;

    // Status variables (and others)
    private boolean limitSwitchAtCurrentCheck;
    private boolean limitSwitchAtLastCheck;
    private boolean zeroed = false;

    private double elevatorSupposedPosition = 0;
    
    public ElevatorSubsystem() {
        // Assign intake motors
        elevatorMotor = new SparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless); //FIXME: find motor id
        elevatorMotorConfig = new SparkMaxConfig();
        softLimitConfig = new SoftLimitConfig();

        // Assign sensors
        limitSwitch = new DigitalInput(ELEVATOR_ZERO_SWITCH_CHANNEL); //FIXME: find port number

        // Assign intake encoder and controller
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPosition(0);
        elevatorController = elevatorMotor.getClosedLoopController();

        configMotors();
        holdElevatorPosition();
    }

    public void teleopInit() {
        setElevatorDutyCycle(0);
    }

    @Override
    public void periodic() {
        updateTelemetry();
        checkSensors();
        checkLimitSwitch();
    }

    public void checkSensors() {
        
    }

    public void updateTelemetry() {
        // Motor publishing
        elevatorMotorPositionPublisher.set(elevatorEncoder.getPosition());
        elevatorMotorSupposedPositionPublisher.set(elevatorSupposedPosition);
        elevatorMotorDutyCyclePublisher.set(elevatorEncoder.getVelocity());
        // Sensor publishing
        atLimitSwitchPublisher.set(getLimitSwitchAtCurrentCheck());
        zeroedPublisher.set(getZeroed());
    }

    public double getElevatorVelocity() {
        return elevatorEncoder.getVelocity();
    }

    private void setSoftLimits() {
        softLimitConfig
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);
        elevatorMotorConfig.apply(softLimitConfig);
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configMotors() {
        // Intake motors
        elevatorMotorConfig
            .inverted(true)
            .smartCurrentLimit(80);

        elevatorMotorConfig.closedLoop
            .pidf(elevatorPIDGains.p, elevatorPIDGains.i, elevatorPIDGains.d, elevatorPIDGains.FF)
            .maxMotion.maxAcceleration(elevatorMaxAccel)
            .maxVelocity(elevatorMaxVelocity)
            .allowedClosedLoopError(elevatorAllowedClosedLoopError);
        
        softLimitConfig
            .forwardSoftLimit(ELEVATOR_MAX_POSITION)
            .reverseSoftLimit(ELEVATOR_MIN_POSITION)
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);
        
        elevatorMotorConfig.apply(softLimitConfig);
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void checkLimitSwitch() {
        limitSwitchAtCurrentCheck = getLimitSwitchAtCurrentCheck();
        if (limitSwitchAtCurrentCheck && !limitSwitchAtLastCheck) {
            elevatorEncoder.setPosition(ELEVATOR_MIN_POSITION);
            if (!zeroed) {
                holdElevatorPosition();
                setSoftLimits();
            }
            zeroed = true;
        }
        limitSwitchAtLastCheck = limitSwitchAtCurrentCheck;
    }

    public void setElevatorSupposedPosition(double position) {
        if (!zeroed) return;
        // Constain the position to the elevator limits
        elevatorSupposedPosition = Math.min(Math.max(position, ELEVATOR_MIN_POSITION), ELEVATOR_MAX_POSITION);
        elevatorController.setReference(position, ControlType.kMAXMotionPositionControl);
    }
    
    public void setElevatorDutyCycle(double dutyCycle) {
        elevatorController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

    public void holdElevatorPosition() {
        setElevatorSupposedPosition(elevatorEncoder.getPosition());
    }

    public Command stowPositionCommand() {
        return this.runOnce(() -> setElevatorSupposedPosition(ELEVATOR_STOW_POSITION));
    }

    public Command grabPositionCommand() {
        return this.runOnce(() -> setElevatorSupposedPosition(ELEVATOR_GRAB_POSITION));
    }

    public Command waitPositionCommand() {
        return this.runOnce(() -> setElevatorSupposedPosition(ELEVATOR_WAIT_POSITION));
    }
    
    public Command l1PositionCommand() {
        return this.runOnce(() -> setElevatorSupposedPosition(ELEVATOR_L1_POSITION));
    }

    public Command l2PositionCommand() {
        return this.runOnce(() -> setElevatorSupposedPosition(ELEVATOR_L2_POSITION));
    }

    public Command l3PositionCommand() {
        return this.runOnce(() -> setElevatorSupposedPosition(ELEVATOR_L3_POSITION));
    }

    public Command l4PositionCommand() {
        return this.runOnce(() -> setElevatorSupposedPosition(ELEVATOR_L4_POSITION));
    }

    public Command clearIntakePositionCommand() {
        return this.runOnce(() -> {
            if (elevatorEncoder.getPosition() < ELEVATOR_CLEAR_INTAKE_POSITION - elevatorAllowedClosedLoopError) {
                setElevatorSupposedPosition(ELEVATOR_CLEAR_INTAKE_POSITION);
            } else {
                holdElevatorPosition();
            }
        });
    }

    public Command zeroElevatorCommand() { // Code uses this function
        // Activate with one press
        return new SequentialCommandGroup(
            Commands.runOnce(() -> { if (!zeroed) { setElevatorDutyCycle(ELEVATOR_ZEROING_SPEED); } } ), 
            Commands.waitUntil(() -> (zeroed)),
            Commands.runOnce(() -> { setElevatorDutyCycle(0); holdElevatorPosition(); })
        ).onlyIf(() -> (!zeroed));
    }

    public Command holdButtonZeroElevatorCommand() { // Code doesn't use this function!
        // Press and hold button to use
        return Commands.runEnd(
            () -> { setElevatorDutyCycle(ELEVATOR_ZEROING_SPEED); }, 
            () -> { setElevatorDutyCycle(0); holdElevatorPosition(); }, this
            ).until(() -> (zeroed)).onlyIf(() -> (!zeroed));
    }

    public boolean atCommandedPosition() {
        return Math.abs(elevatorEncoder.getPosition() - elevatorSupposedPosition) < elevatorAllowedClosedLoopError;
    }

    public boolean getLimitSwitchAtCurrentCheck() {
        return !limitSwitch.get();
    }

    public boolean getZeroed() {
        return zeroed;
    }
}
