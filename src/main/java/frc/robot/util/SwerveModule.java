package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {

    /**
     * Publishes module data to NetworkTables
     */
    public void updateTelemetry();

    /**
     * Runs the drive motor at a specified voltage with module rotated forward
     * 
     * @param voltage Voltage to run drive motor
     */
    public void runForward(double voltage);

    /**
     * Runs the steer motor at a specified voltage with the drive motor stopped
     * 
     * @param voltage Voltage to run steer motor
     */
    public void runRotation(double voltage);
    
    /**
     * Returns the current state of the module.
     * <p>
     * The state includes the velocity and angle of the wheel.
     * 
     * @return The current state of the module.
     */
    public SwerveModuleState getState();

    /**
     * Returns the current position of the module.
     * <p>
     * The position includes the position and angle of the wheel.
     * 
     * @return The current position of the module.
     */
    public SwerveModuleState getDesiredState();

     /**
     * Returns the desired (set) state of the module
     * <p>
     * The state includes the desired velocity and angle of the wheel.
     * 
     * @return The desired state of the module
     */
    public SwerveModulePosition getPosition();

    /**
     * Sets the desired state for the module.
     * 
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState);

    /**
     * Runs the control loops for the motors
     * <p>
     * Should be called by drivetrain subsystem periodic method
     */
    public void periodic();

     /**
     * Returns a Rotation2d object representing the current rotation of the module, as measured by the integrated encoder and adjusted by the steerOffset.
     * <p>
     * 
     * @return A Rotation2d object representing the current rotation of the module.
     */
    public Rotation2d getModuleRotation();

    /**
     * Returns a Rotation2d object representing the current rotation of the module, as measured by the absolute encoder and adjusted by the steerOffset.
     * <p>
     * The rotation of the module will be in the range [-pi, pi] radians.
     * 
     * @return A Rotation2d object representing the current rotation of the module.
     */
    public Rotation2d getAbsoluteModuleRotation();
}