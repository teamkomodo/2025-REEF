package frc.robot.subsystems;

import static frc.robot.Constants.HELICOPTER_MOTOR_ID;
import static frc.robot.Constants.HELICOPTER_SENSOR_CHANNEL;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HelicopterSubsystem extends SubsystemBase {
  // NetworkTable publishers
	private final NetworkTable helicopterTable = NetworkTableInstance.getDefault().getTable("helicoper");
	private final DoublePublisher armAnglePublisher = helicopterTable.getDoubleTopic("armAnglePublisher").publish();

  private final SparkMax helicopterMotor;
  private final SparkMaxConfig helicopterMotorConfig;

  private final RelativeEncoder helicopterEncoder;
  private final SparkClosedLoopController helicopterController;

  private final DigitalInput helicopterLimitSwitch;




  public HelicopterSubsystem() { // CONSTRUCTION

    helicopterMotor = new SparkMax(HELICOPTER_MOTOR_ID, SparkMax.MotorType.kBrushless);
    helicopterMotorConfig = new SparkMaxConfig();

    helicopterEncoder = helicopterMotor.getEncoder();
    helicopterEncoder.setPosition(0);
    helicopterController = helicopterMotor.getClosedLoopController();

    //Sensors

    helicopterLimitSwitch = new DigitalInput(HELICOPTER_SENSOR_CHANNEL);

    configMotors();



  }

  @Override
  public void periodic() {
    checkSensors();
    updateTelemetry();
  }

  private void updateTelemetry() {
    armAnglePublisher.set(helicopterEncoder.getPosition());
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
