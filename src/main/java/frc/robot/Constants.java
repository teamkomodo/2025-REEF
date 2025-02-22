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
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
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
  public static final double DRIVETRAIN_WIDTH = 0.57785; // Distance between center of left and right swerve wheels in meters
  public static final double DRIVETRAIN_LENGTH = 0.57785; // Distance between center of front and back swerve wheels in meters

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
  public static final int INTAKE_HINGE_MOTOR_ID = 11;
  public static final int INTAKE_HINGE_MOTOR_2_ID = 12;
    // Intake and hinge sensor channels
  public static final int CORAL_INTAKE_SENSOR_CHANNEL = 0;
  public static final int CORAL_INTAKE_SENSOR_2_CHANNEL = 1;
  public static final int INTAKE_HINGE_ZERO_SWITCH_CHANNEL = 7;
    // Intake speed and gear ratio
  public static final double INTAKE_SPEED = 0.4; // FIXME: Find best value, 0.3 might be better
  public static final double INTAKE_HINGE_GEAR_RATIO = 45;
  public static final double INTAKE_HINGE_ZEROING_SPEED = -0.1;
    // Intake hinge positions
  public static final double INTAKE_HINGE_MIN_POSITION = 0;
  public static final double INTAKE_HINGE_MAX_POSITION = 13.547;
  public static final double INTAKE_HINGE_STOW_POSITION = 1.2;
  public static final double INTAKE_HINGE_CLEAR_CORAL_POSITION = 3.4;
  public static final double INTAKE_HINGE_SAFE_ELEVATOR_POSITION  = 2;
  public static final double INTAKE_HINGE_START_POSITION = 0; // 0 is all the way up
  public static final double INTAKE_HINGE_STATION_INTAKE_POSITION = 4.238;
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
  public static final double ELEVATOR_WAIT_POSITION = 13.31;
  public static final double ELEVATOR_STOW_POSITION = 3.25;
  public static final double ELEVATOR_GRAB_POSITION = 9.9;
  public static final double ELEVATOR_CLEAR_INTAKE_POSITION = 13.5;
  
  public static final double ELEVATOR_L1_POSITION = 7.54; // FIXME: These are not tuned!
  public static final double ELEVATOR_L2_POSITION = 9.53; // They are kind of close though.
  public static final double ELEVATOR_L3_POSITION = 17.54;
  public static final double ELEVATOR_L4_POSITION = 29.75;

  // Indexer
    // Indexer motor IDs
  public static final int INDEXER_LEFT_BELT_MOTOR_ID = 16;
  public static final int INDEXER_RIGHT_BELT_MOTOR_ID = 15;
    // Indexer sensor channels
  public static final int INDEXER_END_SENSOR_CHANNEL = 3;
  public static final int INDEXER_START_SENSOR_CHANNEL = 2;
    // Indexer motor proportions
  public static final double INDEXER_LEFT_BELT_SPEED_PROPORTION = 1.0; // FIXME: Correct proportions
  public static final double INDEXER_RIGHT_BELT_SPEED_PROPORTION = 0.8; // These are all relative to the same value
  public static final double INDEXER_CENTERING_SPEED_PROPORTION = 0.7;

  // Helicopter
    // Motor IDs
  public static final int HELICOPTER_MOTOR_ID = 41;
    // Gear ratio and offset
  public static final double HELICOPTER_GEAR_RATIO = 47.25;
  public static final double HELICOPTER_OFFSET = 0.1171;
  public static final double HELICOPTER_MIN_POSITION = 0.6;
  public static final double HELICOPTER_MAX_POSITION = 1.07;
    // Helicopter positions
  public static final double HELICOPTER_STOW_POSITION = 0.6; // FIXME: Find actual values
  public static final double HELICOPTER_GRAB_WAIT_POSITION = 1.061;
  public static final double HELICOPTER_LOW_ALGAE_POSITION = 0.6;
  public static final double HELICOPTER_HIGH_ALGAE_POSITION = 0.75;
  public static final double HELICOPTER_WAIT_FOR_ELEVATOR_POSITION = 0.6;
  public static final double HELICOPTER_ZERO_ELEVATOR_POSITION = 0.6;
  public static final double HELICOPTER_RELEASE_CORAL_POSITION = 0.77;
  public static final double HELICOPTER_REMOVE_ALGAE_POSITION = 0.79;
  public static final double HELICOPTER_L1_POSITION = 0.8;
  public static final double HELICOPTER_L2_L3_POSITION = 0.7;
  public static final double HELICOPTER_L4_POSITION = 0.76;

  // End Effector
  public static final int ENDEFFECTOR_MOTOR_ID = 10;
  public static final int ENDEFFECTOR_SENSOR_CHANNEL = 4;


  public static final double WHEEL_DIAMETER = 0.1016;

    /**
     * motor rotations -> wheel rotations
     */
  public static final double DRIVE_REDUCTION = /*(14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)*/ 1/6.75;
    

    /*
    * motor rotations -> module rotations
    */
  public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

  public static double MAX_ATTAINABLE_VELOCITY = 4.5;

  public static final double LINEAR_VELOCITY_CONSTRAINT = MAX_ATTAINABLE_VELOCITY;
  public static final double LINEAR_ACCEL_CONSTRAINT = 12.0;

  public static final double ANGULAR_VELOCITY_CONSTRAINT = (LINEAR_VELOCITY_CONSTRAINT * Math.PI) / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH) * 0.8;
  public static final double ANGULAR_ACCEL_CONSTRAINT = (LINEAR_ACCEL_CONSTRAINT * Math.PI) / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH);


  public static final PPHolonomicDriveController HOLONOMIC_PATH_FOLLOWER_CONFIG = 
  new PPHolonomicDriveController/*PPHolonomicPathFollowerConfig*/ (
    new PIDConstants(2, 0, 0),
    new PIDConstants(2, 0, 0),
    Math.sqrt(DRIVETRAIN_LENGTH*DRIVETRAIN_LENGTH + DRIVETRAIN_WIDTH*DRIVETRAIN_WIDTH)/2
  );



  



  

  public static final BooleanSupplier ON_RED_ALLIANCE = () -> {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  };
}
