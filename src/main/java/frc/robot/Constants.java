// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean TUNING_MODE = false;

  // Controls
  public static final double XBOX_DEADBAND = 0.06;
  public static final int DRIVER_XBOX_PORT = 0;
  public static final int OPERATOR_XBOX_PORT = 1;

  public static final double LINEAR_SLOW_MODE_MODIFIER = 0.5;
  public static final double ANGULAR_SLOW_MODE_MODIFIER = 0.3;
  public static final double DRIVETRAIN_WIDTH = 0.57785; // Distance between center of left and right swerve wheels in
                                                         // meters
  public static final double DRIVETRAIN_LENGTH = 0.57785; // Distance between center of front and back swerve wheels in
                                                          // meters

  public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 32;
  public static final int BACK_RIGHT_STEER_MOTOR_ID = 34;
  public static final int BACK_RIGHT_STEER_ENCODER_ID = 22;
  public static final double BACK_RIGHT_STEER_OFFSET = 5.277;

  public static final int BACK_LEFT_DRIVE_MOTOR_ID = 39;
  public static final int BACK_LEFT_STEER_MOTOR_ID = 38;
  public static final int BACK_LEFT_STEER_ENCODER_ID = 21;   
  public static final double BACK_LEFT_STEER_OFFSET = 1.089;

  public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 33;
  public static final int FRONT_RIGHT_STEER_MOTOR_ID = 35;
  public static final int FRONT_RIGHT_STEER_ENCODER_ID = 20;  
  public static final double FRONT_RIGHT_STEER_OFFSET = 1.2796;

  public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 36;
  public static final int FRONT_LEFT_STEER_MOTOR_ID = 37;
  public static final int FONT_LEFT_STEER_ENCODER_ID = 23;
  public static final double FRONT_LEFT_STEER_OFFSET = 1.7006;

  // Intake
    // Intake motor IDs
  public static final int INTAKE_MOTOR_ID = 51;
  public static final int INTAKE_HINGE_MOTOR_ID = 12;
    // Intake and hinge sensor channels
  public static final int CORAL_INTAKE_SENSOR_CHANNEL = 0;
  public static final int CORAL_INTAKE_SENSOR_2_CHANNEL = 2;
  public static final int INTAKE_HINGE_ZERO_SWITCH_CHANNEL = 7;
    // Intake speed and gear ratio
  public static final double INTAKE_SPEED = 0.55;
  public static final double SLOW_INTAKE_SPEED = 0.40;
  public static final double INTAKE_HINGE_GEAR_RATIO = 45;
  public static final double INTAKE_HINGE_ZEROING_SPEED = -0.2;
    // Intake hinge positions
  public static final double INTAKE_HINGE_MIN_POSITION = 0;
  public static final double INTAKE_HINGE_MAX_POSITION = 13.547;
  public static final double INTAKE_HINGE_STOW_POSITION = 1.5;
  public static final double INTAKE_HINGE_FEED_CORAL_POSITION = 9.0; // This is for when there is a coral in the intake and we want to help feed it through
  public static final double INTAKE_HINGE_SAFE_ELEVATOR_POSITION  = 2;
  public static final double INTAKE_HINGE_START_POSITION = 0; // 0 is all the way up
  public static final double INTAKE_HINGE_CLEAR_ARM_POSITION = 6.0;
  public static final double INTAKE_HINGE_INTAKE_POSITION = 13.547; // This is flat on the ground
  
  // Elevator
    // Motor ID
  public static final int ELEVATOR_MOTOR_ID = 40;
  public static final int ELEVATOR_ZERO_SWITCH_CHANNEL = 5;
    // Zeroing speed
  public static final double ELEVATOR_ZEROING_SPEED = -0.1;
    // Elevator positions
  public static final double ELEVATOR_MIN_POSITION = 0; // Min position is the limit switch position
  public static final double ELEVATOR_MAX_POSITION = 29.75;
  public static final double ELEVATOR_WAIT_POSITION = 13.0; // Was 14.31
  public static final double ELEVATOR_STOW_POSITION = 13;

  public static final double ELEVATOR_FLOOR_ALGAE_POSITION = 13;
  public static final double ELEVATOR_LOW_ALGAE_POSITION = 16;
  public static final double ELEVATOR_HIGH_ALGAE_POSITION = 27;
  public static final double ELEVATOR_GRAB_POSITION = 7.902;
  public static final double ELEVATOR_CLEAR_INTAKE_POSITION = 12.477;
  
  public static final double ELEVATOR_SCORE_ALGAE_POSITION = 29;
  public static final double ELEVATOR_L1_POSITION = 7.54;
  public static final double ELEVATOR_L2_POSITION = 9.1;
  public static final double ELEVATOR_L3_POSITION = 16.896;
  public static final double ELEVATOR_L4_POSITION = 29.75;

  // Indexer
    // Indexer sensor channels
  public static final int INDEXER_END_SENSOR_CHANNEL = 3;
  public static final int INDEXER_START_SENSOR_CHANNEL = 1;

  // Helicopter
    // Motor IDs
  public static final int HELICOPTER_MOTOR_ID = 41;
    // Gear ratio and offset
  public static final double HELICOPTER_GEAR_RATIO = 47.25;
  public static final double HELICOPTER_OFFSET = 0.1171;
  public static final double HELICOPTER_MIN_POSITION = 0.60;
  public static final double HELICOPTER_MAX_POSITION = 1.09;
    // Helicopter positions
  public static final double HELICOPTER_STOW_POSITION = 0.6;
  public static final double HELICOPTER_GRAB_POSITION = 1.104;
  public static final double HELICOPTER_WAIT_POSITION = 0.98;
  public static final double HELICOPTER_LOW_ALGAE_POSITION = 0.8;
  public static final double HELICOPTER_HIGH_ALGAE_POSITION = 0.75;
  public static final double HELICOPTER_FLOOR_ALGAE_POSITION = 0.9;
  public static final double HELICOPTER_WAIT_FOR_L4_POSITION = 0.65;
  public static final double HELICOPTER_WAIT_FOR_L3_POSITION = 0.688;
  public static final double HELICOPTER_MIN_SAFE_POSITION = 0.53;
  public static final double HELICOPTER_MAX_SAFE_POSITION = 0.77;
  public static final double HELICOPTER_RELEASE_CORAL_POSITION = 0.8;
  public static final double HELICOPTER_SCORE_ALGAE_POSITION = 0.6; // FIXME: TUNE ME
  public static final double HELICOPTER_L1_POSITION = 0.73;
  public static final double HELICOPTER_L2_L3_POSITION = 0.718;
  public static final double HELICOPTER_L4_POSITION = 0.76;

  // End Effector
  public static final int ENDEFFECTOR_MOTOR_ID = 10;
  public static final int ENDEFFECTOR_SENSOR_CHANNEL = 4;


  public static final double WHEEL_DIAMETER = 0.1016;

  /**
   * motor rotations -> wheel rotations
   */
  public static final double DRIVE_REDUCTION = /* (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0) */ 1 / 6.75;

  /*
   * motor rotations -> module rotations
   */
  public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

  public static double MAX_ATTAINABLE_VELOCITY = 4.5;

  public static final double LINEAR_VELOCITY_CONSTRAINT = MAX_ATTAINABLE_VELOCITY;
  public static final double LINEAR_ACCEL_CONSTRAINT = 12.0;

  public static final double ANGULAR_VELOCITY_CONSTRAINT = (LINEAR_VELOCITY_CONSTRAINT * Math.PI) / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) * 0.8;
  public static final double ANGULAR_ACCEL_CONSTRAINT = (LINEAR_ACCEL_CONSTRAINT * Math.PI) / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) * 0.5;

  public static final boolean FIELD_RELATIVE_DRIVE = true;
  public static final boolean ALIGNMENT_DRIVE = false;
  public static final int FRONT_LEFT_STEER_ENCODER_ID = 10;

  public static final double LIMELIGHT_X_OFFSET = 0;

  public static final double MAX_MODULE_VELOCITY = 4.058; // physical maximum attainable speed of swerve modules
  public static final double MAX_MODULE_ACCEL = 21; // physical maximum attainable accel of swerve modules

  public static final double MAX_ANGULAR_VELOCITY = 4.0 * Math.PI; // constraint for angular velocity
  public static final double MAX_ANGULAR_ACCEL = 4.0 * Math.PI; // constraint for angular acceleration

  public static final double FALCON_500_NOMINAL_VOLTAGE = 12.0;
  public static final double TALON_FX_TICKS_PER_ROTATION = 2048.0;

  public static final PIDConstants DRIVE_PID = new PIDConstants(5, 0, 0);
  public static final PIDConstants STEER_PID = new PIDConstants(5, 0, 0);

  public static PPHolonomicDriveController HOLONOMIC_PATH_FOLLOWER_CONFIG = new PPHolonomicDriveController(
      DRIVE_PID, // Translation Constants
      STEER_PID, // Steering Constants
      Math.sqrt(DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) / 2);
  public static final int LED_CHANNEL = 0;

  // FRC Field
  public static final double FIELD_WIDTH = 8.21; // approximation: Field Length is 26ft. 11 1/8 in wide
  public static final double FIELD_LENGTH = 16.54;
  public static final double APRILTAG_TO_BRANCH_X_DISTANCE = 0.1651; // horizontal distance from april tag to branch in
                                                                     // meters

  // Vision
  public static final double LIMELIGHT_TO_APRILTAG_Y_DISTANCE = 0.6604; //UPDATE BEFORE SCRIMMAGE
  public static final double LIMELIGHT_ROBOT_X_OFFSET = 0; //UPDATE BEFORE SCRIMMAGE
  public static final double ROBOT_ALIGNMENT_SPEED = 1.0; 
  public static final double ALIGN_LINEAR_SPEED_FACTOR = 1.1;
  public static final double ALIGN_EXPONENTIAL_SPEED_FACTOR = 0.9;
  public static final double ALIGN_TURN_CONSTANT = 0.3;

  public static final double APRILTAG_HEIGHT = 0.5715;
  public static final double LIMELIGHT_ROBOT_Y_OFFSET = -0.0254; //UPDATE BEFORE SCRIMMAGE
  public static final double LIMELIGHT_HEIGHT = 0.1778; //UPDATE BEFORE SCRIMMAGE
  public static final double LIMELIGHT_ANGLE_OFFSET = 28.3; //UPDATE BEFORE SCRIMMAGE (degrees)
  public static final double TY_ALIGN_THRESHOLD = -3; 

  public static final BooleanSupplier ON_RED_ALLIANCE = () -> {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  };

 


  
}
