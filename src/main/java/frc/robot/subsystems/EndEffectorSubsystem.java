package frc.robot.subsystems;

import static frc.robot.Constants.ENDEFFECTOR_MOTOR_ID;
import static frc.robot.Constants.ENDEFFECTOR_SENSOR_CHANNEL;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final SparkMax endEffectorMotor;
  private final SparkMaxConfig endEffectorMotorConfig;

  private final RelativeEncoder endEffectorEncoder;
  private final SparkClosedLoopController endEffectorController;

  private final DigitalInput coralLoadedSensor;




  public EndEffectorSubsystem() { // CONSTRUCTION

    endEffectorMotor = new SparkMax(ENDEFFECTOR_MOTOR_ID, SparkMax.MotorType.kBrushless);
    endEffectorMotorConfig = new SparkMaxConfig();

    endEffectorEncoder = endEffectorMotor.getEncoder();
    endEffectorEncoder.setPosition(0);
    endEffectorController = endEffectorMotor.getClosedLoopController();

    //Sensors

    coralLoadedSensor = new DigitalInput(ENDEFFECTOR_SENSOR_CHANNEL);

    configMotors();



  }

  private void configMotors() {
    endEffectorMotorConfig
        .inverted(false)
        .smartCurrentLimit( 50); //FIXME: set current limit

    endEffectorMotorConfig.closedLoop
        .pid(1, 0, 0); //FIXME: set PID values

    endEffectorMotor
        .configure(endEffectorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }

  public void setEndEffectorDutyCycle(double dutyCycle) {
    endEffectorController.setReference(dutyCycle, ControlType.kDutyCycle);

  }

  public boolean getCoralDetection(DigitalInput beamBreak) {
    return beamBreak.get();
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

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
