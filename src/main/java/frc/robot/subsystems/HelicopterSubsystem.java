package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDGains;

import static frc.robot.Constants.*;

public class HelicopterSubsystem extends SubsystemBase {
    // NetworkTable publishers
    private final NetworkTable helicopterTable = NetworkTableInstance.getDefault().getTable("helicopter");
    private final DoublePublisher armAnglePublisher = helicopterTable.getDoubleTopic("armAnglePublisher").publish();
    private final DoublePublisher armTargetAnglePublisher = helicopterTable.getDoubleTopic("armTargetAnglePublisher").publish();
    private final DoublePublisher armAbsoluteEncoderPublisher = helicopterTable.getDoubleTopic("armAbsoluteEncoderPosition").publish();
    private final BooleanPublisher atCommandedPositionPublisher = helicopterTable.getBooleanTopic("atCommandedPosition").publish();
    private final BooleanPublisher safeForElevatorPublisher = helicopterTable.getBooleanTopic("armPositionSafeForElevator").publish();
    private final IntegerPublisher positionWaitingOnPublisher = helicopterTable.getIntegerTopic("positionWaitingOnPublisher").publish();

    // Motors
    private final SparkMax helicopterMotor;
    private final SparkMaxConfig helicopterMotorConfig;
    private final SoftLimitConfig softLimitConfig;

    // private double helicopterOffsetDifference = 0;

    private final RelativeEncoder helicopterMotorEncoder;
    private final SparkClosedLoopController helicopterController;

    private final SparkAbsoluteEncoder helicopterAbsoluteEncoder;

    // PID Constants
    public double heliP = 0.1;
    public double heliI = 0.0;
    public double heliD = 0.3;

    public PIDGains helicopterPIDGains = new PIDGains(heliP, heliI, heliD, 0.0); //0.1, 0.0000001  , 0.05, 0.0
    private final double helicopterMaxAccel = 1500;
    private final double helicopterMaxVelocity = 6000;
    private final double helicopterAllowedClosedLoopError = 0.15; // This is in arm motor rotations, not arm rotations
    
    // Variables
    private double targetAngle = 0;
    private int positionWaitingOn = 0;

    public HelicopterSubsystem() { // CONSTRUCTION
        // Motor
        helicopterMotor = new SparkMax(HELICOPTER_MOTOR_ID, SparkMax.MotorType.kBrushless);
        helicopterMotorConfig = new SparkMaxConfig();
        softLimitConfig = new SoftLimitConfig();

        // Encoder
        helicopterMotorEncoder = helicopterMotor.getEncoder();
        helicopterMotorEncoder.setPosition(0);

        // Absolute encoder
        helicopterAbsoluteEncoder = helicopterMotor.getAbsoluteEncoder();

        // PID controller
        helicopterController = helicopterMotor.getClosedLoopController();

        configMotors();
    }

    public void teleopInit() {
        setHelicopterPosition(HELICOPTER_STOW_POSITION);
    }

    @Override
    public void periodic() {
        checkSensors();
        updateTelemetry();
        //updatePID();
    }

    private void updateTelemetry() {
        armAnglePublisher.set(helicopterMotorEncoder.getPosition() / HELICOPTER_GEAR_RATIO);
        armTargetAnglePublisher.set(targetAngle);
        atCommandedPositionPublisher.set(atCommandedPosition());
        positionWaitingOnPublisher.set(positionWaitingOn);
        safeForElevatorPublisher.set(isSafeForElevator());
        armAbsoluteEncoderPublisher.set(getAbsoluteEncoderPosition());
    }

    private void checkSensors() {
        // Empty!
    }

    private void configMotors() {
        // Fill out the config
        helicopterMotorConfig
            .inverted(false)
            .smartCurrentLimit(50);

        helicopterMotorConfig.closedLoop
            .pidf(helicopterPIDGains.p, helicopterPIDGains.i, helicopterPIDGains.d, helicopterPIDGains.FF)
            .maxMotion.maxAcceleration(helicopterMaxAccel)
            .maxVelocity(helicopterMaxVelocity)
            .allowedClosedLoopError(helicopterAllowedClosedLoopError);
        
        // Fill out the soft limit config
        softLimitConfig
            .forwardSoftLimit(HELICOPTER_MAX_POSITION * HELICOPTER_GEAR_RATIO)
            .reverseSoftLimit(HELICOPTER_MIN_POSITION * HELICOPTER_GEAR_RATIO)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        // Apply the configs
        helicopterMotorConfig.apply(softLimitConfig);
        helicopterMotor
            .configure(helicopterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Tell the motor relative encoder where it is 
        helicopterMotorEncoder.setPosition(getAbsoluteEncoderPosition() * HELICOPTER_GEAR_RATIO);
    }

    public void setHelicopterPosition(double position) {
        targetAngle = position; // Save the last commanded position
        positionWaitingOn = 0; // We are not currently aligning to a branch (but if we are this will be overridden later)
        helicopterMotorEncoder.setPosition(getAbsoluteEncoderPosition() * HELICOPTER_GEAR_RATIO); // Reset the relative encoder
        helicopterController.setReference(
            targetAngle * HELICOPTER_GEAR_RATIO, 
            ControlType.kMAXMotionPositionControl); // COMMAND THE MOTOR!!!
    }

    public void holdMotorPosition() {
        setHelicopterPosition(getAbsoluteEncoderPosition()); // Freeze!
    }

    // HIGHLY REPETITIVE POSITION COMMANDS
    public Command stowPositionCommand() {
        return this.runOnce(() -> setHelicopterPosition(HELICOPTER_STOW_POSITION));
    }
    
    public Command waitPositionCommand() {
        return this.runOnce(() -> setHelicopterPosition(HELICOPTER_WAIT_POSITION));
    }
    
    public Command grabPositionCommand() {
        return this.runOnce(() -> setHelicopterPosition(HELICOPTER_GRAB_POSITION));
    }
    
    public Command ejectCoralPositionCommand() {
        return this.runOnce(() -> setHelicopterPosition(HELICOPTER_EJECT_CORAL_POSITION));
    }

    public Command scoreAlgaePositionCommand() {
        return this.runOnce(() -> setHelicopterPosition(HELICOPTER_SCORE_ALGAE_POSITION));
    }
    
    public void waitForL4Position() {
        setHelicopterPosition(HELICOPTER_WAIT_FOR_L4_POSITION);
    }

    public void waitForL1L2L3Position() {
        setHelicopterPosition(HELICOPTER_WAIT_FOR_L2_L3_POSITION); // Setting the position sets positionWaitingOn to 0 (none)
    }

    // Position commands start
    public Command safeElevatorPositionCommand() {
        return Commands.runOnce(() -> setHelicopterPosition(HELICOPTER_SAFE_ELEVATOR_POSITION));
    }

    // Level position commands, these override the current level waiting for to keep track of that information.
    public Command l1WaitPositionCommand() {
        return this.runOnce(() -> {
            waitForL1L2L3Position(); // positionWaiting on gets set to zero
            positionWaitingOn = 1;
        });
    }

    public Command l2WaitPositionCommand() {
        return this.runOnce(() -> {
            waitForL1L2L3Position(); // positionWaiting on gets set to zero
            positionWaitingOn = 2;
        });
    }

    public Command l3WaitPositionCommand() {
        return this.runOnce(() -> {
            waitForL1L2L3Position(); // positionWaiting on gets set to zero
            positionWaitingOn = 3;
        });
    }

    public Command l4WaitPositionCommand() {
        return this.runOnce(() -> {
            waitForL4Position(); // positionWaiting on gets set to zero
            positionWaitingOn = 4;
        });
    }

    public Command releaseCoralPositionCommand() {
        return this.runOnce(() -> {
            if (positionWaitingOn > 1) { // Lets not do this if we're at L1 and would hit the reef
                setHelicopterPosition(0.95);
            }
        });
    }
    
    // This uses the remembered waiting position to know how far to go down to score
    public Command scoreCommand() {
        return this.runOnce(() -> {
            if (positionWaitingOn == 1) {
                setHelicopterPosition(HELICOPTER_L1_POSITION);
            } else if (positionWaitingOn == 2 || positionWaitingOn == 3) { // L2 and L3 positions are the same
                setHelicopterPosition(HELICOPTER_L2_L3_POSITION);
            } else if (positionWaitingOn == 4) {
                setHelicopterPosition(HELICOPTER_L4_POSITION);
            }
        });
    }

    // Return the absolute encoder position after adding the offset
    // You should not access the absolute encoder without using this!
    private double getAbsoluteEncoderPosition() {
        return helicopterAbsoluteEncoder.getPosition() + HELICOPTER_OFFSET;
    }

    // Returns a boolean indicating if the motor is at the commanded position
    public boolean atCommandedPosition() {
        return Math.abs(helicopterMotorEncoder.getPosition() - targetAngle * HELICOPTER_GEAR_RATIO) < helicopterAllowedClosedLoopError;
    }

    // Returns a boolean indicating if the motor is at a position where the arm will not be damaged if the elevator moves
    public boolean isSafeForElevator() {
        return getAbsoluteEncoderPosition() <= HELICOPTER_MAX_SAFE_POSITION && getAbsoluteEncoderPosition() >= HELICOPTER_MIN_SAFE_POSITION;
    }

    // Getter
    public int getPositionWaitingOn() {
        return positionWaitingOn;
    }

}