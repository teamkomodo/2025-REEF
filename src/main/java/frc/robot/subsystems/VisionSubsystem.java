package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Robot;
import frc.robot.util.FFGains;
import frc.robot.util.NeoSwerveModule;
import frc.robot.util.PIDGains;
import frc.robot.util.SwerveModule;
import frc.robot.util.Util;

import static frc.robot.Constants.*;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.VisionSubsystem;

import java.io.Console;
import java.io.IOException;
import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import java.io.File;
import java.io.IOException;

import com.kauailabs.navx.frc.AHRS;




import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.config.ModuleConfig;
// import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
// import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FileVersionException;

public class VisionSubsystem implements Subsystem {

    /*
     * Robot Coordinate System
     * See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
     * Forward is x+, Left is y+, counterclockwise is theta+
     */

    // Limelight
    private static boolean useVision = false;

    private final NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight");
    private final DoubleSubscriber validTargetSubscriber = limelightNT.getDoubleTopic("tv").subscribe(0);
    private final DoubleArraySubscriber botPoseBlueSubscriber = limelightNT.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);

    // Telemetry
    public static final NetworkTable drivetrainNT = NetworkTableInstance.getDefault().getTable("drivetrain");

   

    // Swerve
    
    private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    //private final RobotConfig robotConfig     


    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
    RobotConfig config;
    // private DriveFeedforwards driveFeedforwards = new DriveFeedforwards(
    //     null, 
    //     null, 
    //     null, 
    //     null, 
    //     null);
    private boolean slowMode = false;
    private double rotationOffsetRadians = 0.0;

    private ChassisSpeeds lastCommandedChassisSpeeds = new ChassisSpeeds();

    public VisionSubsystem() {

    }
    
    @Override
    public void periodic() {
        
    }

    
   
    public AHRS getNavx() {
        return navX;
    }
    double limelightX(){
        double xP = 0.014;
        double targetingForwardSpeed = LimelightHelpers.getTX("limelight") * xP;
        targetingForwardSpeed *= 1;
        targetingForwardSpeed *= -3.5;
        
        if(Math.abs(LimelightHelpers.getTX("limelight")) > 0.5){
            return targetingForwardSpeed;
        }
        return 0;
    }


    double limelightY(){
        double yP = .04;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * yP;
        //targetingForwardSpeed *= 1;
        targetingForwardSpeed *= -3;
        
        if(Math.abs(LimelightHelpers.getTY("limelight")) > 0.5){
            return targetingForwardSpeed;
        }
        return 0;

    }
    double limelightZ(){
        double zP = 0.4;
        double targetingZ = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6])[5] *zP;
        targetingZ *= 0.8;
        //if(targetingZ = 0)
        //spin in place
        
        //System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6])[5]);
        if(Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6])[5]) > 0.0){
            return -targetingZ;
        }
        return 0;
        
    }
    public void detectAprilTag(CommandXboxController controller){
            boolean tv = LimelightHelpers.getTV("limelight");

            if(tv){
                controller.setRumble(RumbleType.kRightRumble, 1);
                //System.out.println("RUMBBLEEE");
            } else {
                controller.setRumble(RumbleType.kRightRumble, 0);
            }
        }

    public double calculateAlignDistance(boolean right) {
        double limelightDistance = (APRILTAG_HEIGHT - LIMELIGHT_HEIGHT)
            / Math.tan(Math.toRadians(LIMELIGHT_ANGLE_OFFSET + LimelightHelpers.getTY("limelight")));

        double branchOffset = limelightDistance
            / Math.tan(Math.toRadians(90 - LimelightHelpers.getTX("limelight")))
            + LIMELIGHT_ROBOT_X_OFFSET;

        if(right){
            branchOffset += APRILTAG_TO_BRANCH_X_DISTANCE;
        }
        else {
            branchOffset -= APRILTAG_TO_BRANCH_X_DISTANCE;
        }

        
        return branchOffset;
    }

    public double calculateAlignTime(boolean right) {
        double alignDriveTime = Math.pow((Math.abs(calculateAlignDistance(right)) / ROBOT_ALIGNMENT_SPEED * ALIGN_LINEAR_SPEED_FACTOR), ALIGN_EXPONENTIAL_SPEED_FACTOR);
        return alignDriveTime;
    }

    public double calculateAlignSpeedDirection(boolean right) {
        if(right) {
            if(calculateAlignDistance(true) < 0)
                return ROBOT_ALIGNMENT_SPEED;
            else
                return -ROBOT_ALIGNMENT_SPEED;
        } else {
            if(calculateAlignDistance(false) < 0)
                return ROBOT_ALIGNMENT_SPEED;
            else
                return -ROBOT_ALIGNMENT_SPEED;
        }
    }

    public Command goToBranch(boolean right){
        return Commands.runOnce(() -> {
            if(LimelightHelpers.getTV("limelight")) {
                double alignDriveTime = Math.abs(calculateAlignTime(right));
                double robotAlignmentSpeed = calculateAlignSpeedDirection(right);
                timedDriveCommand(1.10, robotAlignmentSpeed, 0, ALIGNMENT_DRIVE, alignDriveTime);
                //doTheThing(robotAlignmentSpeed, alignDriveTime);
                System.out.println("DO SOMETHING");
            }
                        
                    
        }, this);
    }

    //FIXME: Implement calls to other class
    public void timedDriveCommand(double xSpeed, double ySpeed, double angularVelocity, boolean fieldRelative, double driveTime) {
        new SequentialCommandGroup(
            Commands.run(() -> drive(xSpeed, ySpeed, angularVelocity, fieldRelative), this).withTimeout(driveTime),
            Commands.runOnce(() -> stopMotion())
        ).schedule();
    }

    //FIXME: Implement calls to other class
    public Command limelightCenterandDriveCommand(){
        return Commands.run(() -> {
            drive(limelightY(), limelightX(), -limelightZ(),  false);
        }, this);
    }

    //FIXME: Implement calls to other class
    public Command parallelCommand(){        
        return Commands.run(() -> {
            
            drive(0, 0, -limelightZ(), false);

            System.out.println(limelightZ());

            // if(limelightZ() != 0){
            //     drive(0, 0, -limelightZ(), false);
            // } else {
            //     System.out.println("stop it get some help");
            // }
                
        }, this).until(() -> (Math.abs(NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6])[5]) < 0.5));
        
    }

    //FIXME: Implement calls to other class
    public Command limelightAlignCommand(){
        return Commands.sequence(
           // parallelCommand(),
            limelightCenterandDriveCommand()
        );
    }
}
