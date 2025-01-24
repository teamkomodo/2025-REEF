package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class HelicopterSubsystem extends SubsystemBase {
  // NetworkTable publishers
	private final NetworkTable helicopterTable = NetworkTableInstance.getDefault().getTable("helicoper");
	private final DoublePublisher armAnglePublisher = helicopterTable.getDoubleTopic("armAnglePublisher").publish();
	private final DoublePublisher armCommandAnglePublisher = helicopterTable.getDoubleTopic("armCommandAnglePublisher").publish();
	private final DoublePublisher armTargetAnglePublisher = helicopterTable.getDoubleTopic("armTargetAnglePublisher").publish();
	private final DoublePublisher armAngleOffsetPublisher = helicopterTable.getDoubleTopic("armAngleOffsetPublisher").publish();


  private final SparkMax helicopterMotor;
  private final SparkMaxConfig helicopterMotorConfig;

  private final RelativeEncoder helicopterMotorEncoder;
  private final SparkClosedLoopController helicopterController;

  private final DigitalInput helicopterAngleEncoder;
  private final DutyCycleEncoder helicopterAbsoluteEncoder;

  private double targetAngle = 0;
  private double motorCommandAngle = 0;
  private double angleOffset = 0;


  public HelicopterSubsystem() { // CONSTRUCTION

    helicopterMotor = new SparkMax(HELICOPTER_MOTOR_ID, SparkMax.MotorType.kBrushless);
    helicopterMotorConfig = new SparkMaxConfig();

    helicopterMotorEncoder = helicopterMotor.getEncoder();
    helicopterMotorEncoder.setPosition(0);
    helicopterController = helicopterMotor.getClosedLoopController();

    // Sensor
    helicopterAngleEncoder = new DigitalInput(HELICOPTER_ABSOLUTE_ENCODER_CHANNEL);
    helicopterAbsoluteEncoder = new DutyCycleEncoder(helicopterAngleEncoder);

    configMotors();

  }

  @Override
  public void periodic() {
    checkSensors();
    updateTelemetry();
    updateMotor();
  }

  private void updateTelemetry() {
    armAnglePublisher.set(helicopterMotorEncoder.getPosition());
    armCommandAnglePublisher.set(motorCommandAngle);
    armTargetAnglePublisher.set(targetAngle);
    armAngleOffsetPublisher.set(angleOffset);
  }

	private void checkSensors() {
    
	}

  private void configMotors() {
    helicopterMotorConfig
      .inverted(false)
      .smartCurrentLimit(50); //FIXME: set current limit

    helicopterMotorConfig.closedLoop
      .pid(1, 0, 0); //FIXME: set PID values

    helicopterMotor
      .configure(helicopterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setHelicopterPosition(double position) {
    targetAngle = position;
    updateMotor();
  }

  public void setMotorPosition(double position) {
    helicopterController.setReference(position, ControlType.kPosition);
  }

  public void setMotorDutyCycle(double dutyCycle) {
    helicopterController.setReference(dutyCycle, ControlType.kDutyCycle);
  }

  private void updateMotor() {
    angleOffset = helicopterMotorEncoder.getPosition() - helicopterAbsoluteEncoder.get();

    motorCommandAngle = targetAngle + angleOffset;
    setMotorPosition(motorCommandAngle);
  }

  public Command stowWaitCommand() {
    return this.runOnce(() -> setHelicopterPosition(HELICOPTER_STOW_GRAB_POSITION));
  }

  public Command lowAlgaePositionCommand() {
    return this.runOnce(() -> setHelicopterPosition(HELICOPTER_LOW_ALGAE_POSITION));
  }

  public Command highAlgaePositionCommand() {
    return this.runOnce(() -> setHelicopterPosition(HELICOPTER_HIGH_ALGAE_POSITION));
  }

  public Command l1PositionCommand() {
    return this.runOnce(() -> setHelicopterPosition(HELICOPTER_L1_POSITION));
  }

  public Command l2PositionCommand() {
    return this.runOnce(() -> setHelicopterPosition(HELICOPTER_L2_L3_POSITION));
  }

  public Command l3PositionCommand() {
    return this.runOnce(() -> setHelicopterPosition(HELICOPTER_L2_L3_POSITION));
  }

  public Command l4PositionCommand() {
    return this.runOnce(() -> setHelicopterPosition(HELICOPTER_L4_POSITION));
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
}
