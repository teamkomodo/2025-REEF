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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PIDGains;

import static frc.robot.Constants.*;

public class HelicopterSubsystem extends SubsystemBase {
    // NetworkTable publishers
    private final NetworkTable helicopterTable = NetworkTableInstance.getDefault().getTable("helicopter");
    private final DoublePublisher armAnglePublisher = helicopterTable.getDoubleTopic("armAnglePublisher").publish();
    private final DoublePublisher armCommandAnglePublisher = helicopterTable.getDoubleTopic("armCommandAnglePublisher").publish();
    private final DoublePublisher armTargetAnglePublisher = helicopterTable.getDoubleTopic("armTargetAnglePublisher").publish();
    private final DoublePublisher armAngleOffsetPublisher = helicopterTable.getDoubleTopic("armAngleOffsetPublisher").publish();
    private final DoublePublisher armAbsoluteEncoderPublisher = helicopterTable.getDoubleTopic("armAbsoluteEncoderPosition").publish();
    private final BooleanPublisher atCommandedPositionPublisher = helicopterTable.getBooleanTopic("atCommandedPosition").publish();
    private final IntegerPublisher positionWaitingOnPublisher = helicopterTable.getIntegerTopic("positionWaitingOnPublisher").publish();

    // Are we using the absolute encoder? Just for early testing. Might be useful later.
    private final boolean useAbsoluteEncoder = !true;

    // Motors
    private final SparkMax helicopterMotor;
    private final SparkMaxConfig helicopterMotorConfig;
    private final SoftLimitConfig softLimitConfig;

    private final RelativeEncoder helicopterMotorEncoder;
    private final SparkClosedLoopController helicopterController;

    private final SparkAbsoluteEncoder helicopterAbsoluteEncoder;

    // PID Constants
    private final PIDGains helicopterPIDGains = new PIDGains(0.1, 0, 0);
    private final double helicopterMaxAccel = 3000;
    private final double helicopterMaxVelocity = 3000;
    private final double helicopterAllowedClosedLoopError = 0.2; // = +/- 1/2 inch of arm movement

    // Variables
    private double targetAngle = 0;
    private double motorCommandAngle = 0;
    private double angleOffset = 0;
    private int positionWaitingOn = 0;


    public HelicopterSubsystem() { // CONSTRUCTION
        helicopterMotor = new SparkMax(HELICOPTER_MOTOR_ID, SparkMax.MotorType.kBrushless);
        helicopterMotorConfig = new SparkMaxConfig();
        softLimitConfig = new SoftLimitConfig();

        helicopterMotorEncoder = helicopterMotor.getEncoder();
        helicopterMotorEncoder.setPosition(0);

        // Sensor
        if (useAbsoluteEncoder) {
            helicopterAbsoluteEncoder = helicopterMotor.getAbsoluteEncoder();
        } else {
            helicopterAbsoluteEncoder = null;
        }

        // PID controller
        helicopterController = helicopterMotor.getClosedLoopController();

        configMotors();
    }

    @Override
    public void periodic() {
        checkSensors();
        updateTelemetry();
    }

    private void updateTelemetry() {
        //armAnglePublisher.set(helicopterMotorEncoder.getPosition() / HELICOPTER_GEAR_RATIO);
        armCommandAnglePublisher.set(motorCommandAngle);
        armTargetAnglePublisher.set(targetAngle);
        // armAngleOffsetPublisher.set(angleOffset);
        atCommandedPositionPublisher.set(atCommandedPosition());
        positionWaitingOnPublisher.set(positionWaitingOn);
        if (useAbsoluteEncoder)
        armAbsoluteEncoderPublisher.set(getAbsoluteEncoderPosition());
    }

    private void checkSensors() {
        // Empty!
    }

    private void configMotors() {
        helicopterMotorConfig
            .inverted(false)
            .smartCurrentLimit(50); //FIXME: set current limit

        helicopterMotorConfig.closedLoop
            .pid(helicopterPIDGains.p, helicopterPIDGains.i, helicopterPIDGains.d)
            .maxMotion.maxAcceleration(helicopterMaxAccel)
            .maxVelocity(helicopterMaxVelocity)
            .allowedClosedLoopError(helicopterAllowedClosedLoopError);
        
        
        softLimitConfig
            .forwardSoftLimit(ELEVATOR_MAX_POSITION)
            .reverseSoftLimit(ELEVATOR_MIN_POSITION)
            .forwardSoftLimitEnabled(false)
            .reverseSoftLimitEnabled(false);

        helicopterMotorConfig.apply(softLimitConfig);
        helicopterMotor
            .configure(helicopterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void setHelicopterPosition(double position) {
        targetAngle = position;
        positionWaitingOn = 0;
        setMotorPosition(targetAngle * HELICOPTER_GEAR_RATIO);
        if (useAbsoluteEncoder) {
            helicopterMotorEncoder.setPosition(getAbsoluteEncoderPosition() * HELICOPTER_GEAR_RATIO);
        }
    }

    public void setMotorPosition(double position) {
        helicopterController.setReference(position, ControlType.kMAXMotionPositionControl);
    }

    public void setMotorDutyCycle(double dutyCycle) {
        helicopterController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

    public Command stowPositionCommand() {
        return this.runOnce(() -> setHelicopterPosition(HELICOPTER_STOW_POSITION));
    }
    
    public Command grabPositionCommand() {
        return this.runOnce(() -> setHelicopterPosition(HELICOPTER_GRAB_POSITION));
    }

    public Command lowAlgaePositionCommand() {
        return this.runOnce(() -> setHelicopterPosition(HELICOPTER_LOW_ALGAE_POSITION));
    }

    public Command highAlgaePositionCommand() {
        return this.runOnce(() -> setHelicopterPosition(HELICOPTER_HIGH_ALGAE_POSITION));
    }
    
    public Command removeAlgaePositionCommand() {
        return Commands.runOnce(() -> {
            if (positionWaitingOn == 3) {
                setHelicopterPosition(HELICOPTER_LOW_ALGAE_POSITION);
            } else if (positionWaitingOn == 4) {
                setHelicopterPosition(HELICOPTER_HIGH_ALGAE_POSITION);
            }
        });
    }

    public void waitForElevatorPosition() {
        setHelicopterPosition(HELICOPTER_WAIT_FOR_ELEVATOR_POSITION);
    }

    public Command zeroElevatorPositionCommand() {
        return this.runOnce(() -> setHelicopterPosition(HELICOPTER_ZERO_ELEVATOR_POSITION));
    }

    public Command l1WaitPositionCommand() {
        return this.runOnce(() -> {
            waitForElevatorPosition();
            positionWaitingOn = 1;
        });
    }

    public Command l2WaitPositionCommand() {
        return this.runOnce(() -> {
            waitForElevatorPosition();
            positionWaitingOn = 2;
        });
    }

    public Command l3WaitPositionCommand() {
        return this.runOnce(() -> {
            waitForElevatorPosition();
            positionWaitingOn = 3;
        });
    }

    public Command l4WaitPositionCommand() {
        return this.runOnce(() -> {
            waitForElevatorPosition();
            positionWaitingOn = 4;
        });
    }

    public Command releaseCoralPositionCommand() {
        return this.runOnce(() -> {
            if (positionWaitingOn > 1) {
                setHelicopterPosition(HELICOPTER_RELEASE_CORAL_POSITION);
            }
        });
    }

    public Command scoreCommand() {
        return this.runOnce(() -> {
            if (positionWaitingOn == 1) {
                setHelicopterPosition(HELICOPTER_L1_POSITION);
            } else if (positionWaitingOn == 2 || positionWaitingOn == 3) {
                setHelicopterPosition(HELICOPTER_L2_L3_POSITION);
            } else if (positionWaitingOn == 4) {
                setHelicopterPosition(HELICOPTER_L4_POSITION);
            }
        });
    }

    private double getAbsoluteEncoderPosition() {
        return helicopterAbsoluteEncoder.getPosition() + HELICOPTER_OFFSET;
    }

    public boolean atCommandedPosition() {
        return Math.abs(getAbsoluteEncoderPosition() - targetAngle) < helicopterAllowedClosedLoopError;
    }

    public int getPositionWaitingOn() {
        return positionWaitingOn;
    }

}