// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Optional;
import java.util.function.BooleanSupplier;


import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

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
  public static final double DRIVETRAIN_WIDTH = 0.5271; // Distance between center of left and right swerve wheels in meters
  public static final double DRIVETRAIN_LENGTH = 0.5271; // Distance between center of front and back swerve wheels in meters

  public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 32;
  public static final int BACK_RIGHT_STEER_MOTOR_ID = 34;
  public static final int BACK_RIGHT_STEER_ENCODER_ID = 22;
  public static final double BACK_RIGHT_STEER_OFFSET = 5.9;

  public static final int BACK_LEFT_DRIVE_MOTOR_ID = 39;
  public static final int BACK_LEFT_STEER_MOTOR_ID = 38;
  public static final int BACK_LEFT_STEER_ENCODER_ID = 21;   
  public static final double BACK_LEFT_STEER_OFFSET = 1.28;

  public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 33;
  public static final int FRONT_RIGHT_STEER_MOTOR_ID = 35;
  public static final int FRONT_RIGHT_STEER_ENCODER_ID = 20;    public static final double FRONT_RIGHT_STEER_OFFSET = 3.75;

  public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 36;
  public static final int FRONT_LEFT_STEER_MOTOR_ID = 37;
  public static final int FONT_LEFT_STEER_ENCODER_ID = 23;
  public static final double FRONT_LEFT_STEER_OFFSET = 4.71;

  public static final double WHEEL_DIAMETER = 0.1016;

    /**
     * motor rotations -> wheel rotations
     */
  public static final double DRIVE_REDUCTION = /*(14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)*/ 1/6.75;
    

  /*
  * motor rotations -> module rotations
  */
  public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

  public static final double MAX_ATTAINABLE_VELOCITY = 1.5;

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
