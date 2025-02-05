package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    private final PIDGains elevatorPIDGains = new PIDGains(1, 0, 0);
    private final double elevatorMaxAccel = 1000;
    private final double elevatorMaxVelocity = 1000;
    private final double elevatorAllowedClosedLoopError = 1;

    // Sensors
    private final DigitalInput limitSwitch;

    // Status variables (and others)
    private boolean limitSwitchAtCurrentCheck;
    private boolean limitSwitchAtLastCheck;
    private boolean zeroed = false;

    private double elevatorPosition = 0;
    
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
        setElevatorDutyCycle(0);
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
        elevatorMotorSupposedPositionPublisher.set(elevatorPosition);
        elevatorMotorDutyCyclePublisher.set(elevatorEncoder.getVelocity());
        // Sensor publishing
        atLimitSwitchPublisher.set(getLimitSwitchAtCurrentCheck());
        zeroedPublisher.set(getZeroed());
    }

    public double getElevatorVelocity() {
        return elevatorEncoder.getVelocity();
    }

    private void configMotors() {
        // Intake motors
        elevatorMotorConfig
            .inverted(false)
            .smartCurrentLimit(80);

        elevatorMotorConfig.closedLoop
            .pid(elevatorPIDGains.p, elevatorPIDGains.i, elevatorPIDGains.d)
            .maxMotion.maxAcceleration(elevatorMaxAccel)
            .maxVelocity(elevatorMaxVelocity)
            .allowedClosedLoopError(elevatorAllowedClosedLoopError);
        
        softLimitConfig
            .forwardSoftLimit(ELEVATOR_MAX_POSITION)
            .reverseSoftLimit(ELEVATOR_MIN_POSITION)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void checkLimitSwitch() {
        limitSwitchAtCurrentCheck = getLimitSwitchAtCurrentCheck();
        if (limitSwitchAtCurrentCheck && !limitSwitchAtLastCheck) {
            elevatorEncoder.setPosition(ELEVATOR_MIN_POSITION);
            if (!zeroed) {
                setElevatorPosition(ELEVATOR_MIN_POSITION);
            }
            zeroed = true;
        }
        limitSwitchAtLastCheck = limitSwitchAtCurrentCheck;
    }

    public void setElevatorPosition(double position) {
        elevatorPosition = position;
        elevatorController.setReference(position, ControlType.kMAXMotionPositionControl);
    }
    
    public void setElevatorDutyCycle(double dutyCycle) {
        elevatorController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

    public void holdElevatorPosition() {
        setElevatorPosition(elevatorEncoder.getPosition());
    }

    public Command stowPositionCommand() {
        return this.runOnce(() -> setElevatorPosition(ELEVATOR_STOW_GRAB_POSITION));
    }

    public Command grabPositionCommand() {
        return this.runOnce(() -> setElevatorPosition(ELEVATOR_STOW_GRAB_POSITION));
    }

    public Command stowWaitCommand() {
        return this.runOnce(() -> setElevatorPosition(ELEVATOR_WAIT_POSITION));
    }

    public Command lowAlgaePositionCommand() {
        return this.runOnce(() -> setElevatorPosition(ELEVATOR_LOW_ALGAE_POSITION));
    }

    public Command highAlgaePositionCommand() {
        return this.runOnce(() -> setElevatorPosition(ELEVATOR_HIGH_ALGAE_POSITION));
    }
    
    public Command l1PositionCommand() {
        return this.runOnce(() -> setElevatorPosition(ELEVATOR_L1_POSITION));
    }

    public Command l2PositionCommand() {
        return this.runOnce(() -> setElevatorPosition(ELEVATOR_L2_POSITION));
    }

    public Command l3PositionCommand() {
        return this.runOnce(() -> setElevatorPosition(ELEVATOR_L3_POSITION));
    }

    public Command l4PositionCommand() {
        return this.runOnce(() -> setElevatorPosition(ELEVATOR_L4_POSITION));
    }

    public Command zeroElevatorCommand() {
        return Commands.runEnd(() -> { if (!zeroed) { setElevatorDutyCycle(-0.1); } }, () -> { if (!zeroed) { holdElevatorPosition(); } }, this).until(() -> (zeroed));
    }

    public boolean getLimitSwitchAtCurrentCheck() {
        return !limitSwitch.get();
    }

    public boolean getZeroed() {
        return zeroed;
    }
}
