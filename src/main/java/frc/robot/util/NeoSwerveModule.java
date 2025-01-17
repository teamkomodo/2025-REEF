package frc.robot.util;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubParameter;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;

import static frc.robot.Constants.*;


public class NeoSwerveModule implements SwerveModule{


   // private  SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    // Telemetry
    private final DoublePublisher normalizedVelocityError;
    private final DoublePublisher rotationErrorPublisher;
    private final DoublePublisher dutyCyclePublisher;
    private final DoublePublisher velocityPublisher;

    private final DoubleEntry drivekPEntry;
    private final DoubleEntry drivekIEntry;
    private final DoubleEntry drivekDEntry;
    private final DoubleEntry drivekSEntry;
    private final DoubleEntry drivekVEntry;
    private final DoubleEntry drivekAEntry;

    private final DoubleEntry steerkPEntry;
    private final DoubleEntry steerkIEntry;
    private final DoubleEntry steerkDEntry;

    private final SparkMax driveMotor;
    private final SparkMaxConfig driveConfig;
    private final SparkMax steerMotor;
    private final SparkMaxConfig steerConfig;

    public double AbsoluteSensorDiscontinuityPoint =0.5;
    
    
    private final CANcoder steerAbsoluteEncoder;

    private SwerveModuleState desiredState;

    private final RelativeEncoder driveRelativeEncoder;
    private final RelativeEncoder steerRelativeEncoder;

    private final PIDController driveController;

    private final SparkClosedLoopController steerController;
    private final SparkClosedLoopController driverController;
        
    private SimpleMotorFeedforward driveFeedforward; // Gains from SysId Analysis


    private double relativeSteerAdjustment = 0;
    private double angularOffset = 0;
    // private double relativeSteerAdjustmentFactor = 0.1;
   // public Rotation2d angle = Rotation2d.kZero;

    public NeoSwerveModule(int driveMotorId, int steerMotorId, int steerAbsoluteEncoderId, double steerOffset, PIDGains steerPIDGains, PIDGains drivePIDGains, FFGains driveFFGains, NetworkTable moduleNT) {
        this.driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        this.steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);
        this.steerAbsoluteEncoder = new CANcoder(steerAbsoluteEncoderId);
        this.desiredState = new SwerveModuleState(0.0, Rotation2d.fromRadians(0));

        driveController = new PIDController(drivePIDGains.p, drivePIDGains.i, drivePIDGains.d);
        driveFeedforward = new SimpleMotorFeedforward(driveFFGains.kS, driveFFGains.kV, driveFFGains.kA);

        driveRelativeEncoder = driveMotor.getEncoder();

        steerConfig = new SparkMaxConfig();
        driveConfig = new SparkMaxConfig();

        steerAbsoluteEncoder.getConfigurator().apply(
            new MagnetSensorConfigs()
            .withMagnetOffset(steerOffset / (2 * Math.PI))
            .withAbsoluteSensorDiscontinuityPoint(AbsoluteSensorDiscontinuityPoint)); // CANCoder outputs between (-0.5, 0.5)
            //hopefully this has the correct values but idk we ball
        steerRelativeEncoder = steerMotor.getEncoder();
        //steerController = steerMotor.getPIDController();
        steerController = steerMotor.getClosedLoopController();
        driverController = driveMotor.getClosedLoopController();

        configureMotors(steerPIDGains);
     
        // Telemetry
        normalizedVelocityError = moduleNT.getDoubleTopic("normvelocityerror").publish();
        rotationErrorPublisher = moduleNT.getDoubleTopic("rotationerror").publish();
        dutyCyclePublisher = moduleNT.getDoubleTopic("dutycycle").publish();
        velocityPublisher = moduleNT.getDoubleTopic("velocity").publish();
        
        drivekPEntry = moduleNT.getDoubleTopic("tuning/drivekP").getEntry(drivePIDGains.p);
        drivekIEntry = moduleNT.getDoubleTopic("tuning/drivekI").getEntry(drivePIDGains.i);
        drivekDEntry = moduleNT.getDoubleTopic("tuning/drivekD").getEntry(drivePIDGains.d);
        drivekSEntry = moduleNT.getDoubleTopic("tuning/drivekS").getEntry(driveFFGains.kS);
        drivekVEntry = moduleNT.getDoubleTopic("tuning/drivekV").getEntry(driveFFGains.kV);
        drivekAEntry = moduleNT.getDoubleTopic("tuning/drivekA").getEntry(driveFFGains.kA);

        steerkPEntry = moduleNT.getDoubleTopic("tuning/steerkP").getEntry(steerPIDGains.p);
        steerkIEntry = moduleNT.getDoubleTopic("tuning/steerkI").getEntry(steerPIDGains.i);
        steerkDEntry = moduleNT.getDoubleTopic("tuning/steerkD").getEntry(steerPIDGains.d);

        drivekPEntry.set(drivePIDGains.p);
        drivekIEntry.set(drivePIDGains.i);
        drivekDEntry.set(drivePIDGains.d);
        drivekSEntry.set(driveFFGains.kS);
        drivekVEntry.set(driveFFGains.kV);
        drivekAEntry.set(driveFFGains.kA);

        steerkPEntry.set(steerPIDGains.p);
        steerkIEntry.set(steerPIDGains.i);
        steerkDEntry.set(steerPIDGains.d);

        angularOffset = steerOffset;
        

    }

    private void configureMotors(PIDGains steerGains) {
      


        //Drive Motors
        driveConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
        
        // driveMotor.setInverted(false);
        // driveMotor.setIdleMode(IdleMode.kBrake);

        double wheelPositionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION; // motor rotations -> wheel travel in meters
        // driveRelativeEncoder.setPositionConversionFactor(wheelPositionConversionFactor);
        // driveRelativeEncoder.setVelocityConversionFactor(wheelPositionConversionFactor / 60); // motor RPM -> wheel speed in m/s

        driveConfig.encoder
        .positionConversionFactor(wheelPositionConversionFactor)
        .velocityConversionFactor(wheelPositionConversionFactor / 60); // motor RPM -> wheel speed in m/s

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Steer Motor
        steerConfig.inverted(true)
        .idleMode(IdleMode.kBrake);

        steerConfig.encoder
        .positionConversionFactor(2 * Math.PI * STEER_REDUCTION)
        .velocityConversionFactor(2 * Math.PI * STEER_REDUCTION / 60);

        
        
        steerAbsoluteEncoder.setPosition(getAbsoluteModuleRotation().getRadians());

        steerConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingMaxInput(Math.PI)
        .positionWrappingMinInput(-Math.PI)
        .pid(steerGains.p, steerGains.i,steerGains.d);



        steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        

        
        // steerMotor.setInverted(true);
        // steerMotor.setIdleMode(IdleMode.kBrake);

        // steerRelativeEncoder.setPositionConversionFactor(2 * Math.PI * STEER_REDUCTION); // motor rotations -> module rotation in radians
        // steerRelativeEncoder.setVelocityConversionFactor(2 * Math.PI * STEER_REDUCTION / 60); // motor RPM -> module rad/s
        steerRelativeEncoder.setPosition(getModuleRotation().getRadians());

        // steerController.setP(steerGains.p);
        // steerController.setI(steerGains.i);
        // steerController.setD(steerGains.d);


        // I cant find any replacement for this thing and i dont even know what it does
        // if swerve freaks out then we're so done for since i can't find the new one in 2025
     /*    steerController.setPositionPIDWrappingEnabled(true);
        
        steerController.setPositionPIDWrappingMaxInput(Math.PI);
        steerController.setPositionPIDWrappingMinInput(-Math.PI);   */
        
    }

    public void updateTelemetry(){
        normalizedVelocityError.set((desiredState.speedMetersPerSecond - getDriveVelocity()) * Math.signum(desiredState.speedMetersPerSecond));
        rotationErrorPublisher.set(MathUtil.angleModulus(desiredState.angle.getRadians() - getModuleRotation().getRadians()));
        dutyCyclePublisher.set(driveMotor.get());
        velocityPublisher.set(getDriveVelocity(), RobotController.getFPGATime() - 200000);

        if(!TUNING_MODE)
            return;

        double newDrivekP = drivekPEntry.get();
        if(newDrivekP != driveController.getP()) driveController.setP(newDrivekP);

        double newDrivekI = drivekIEntry.get();
        if(newDrivekI != driveController.getI()) driveController.setI(newDrivekI);

        double newDrivekD = drivekDEntry.get();
        if(newDrivekD != driveController.getD()) driveController.setD(newDrivekD);

        double newDrivekS = drivekSEntry.get();
        double newDrivekV = drivekVEntry.get();
        double newDrivekA = drivekAEntry.get();

        if(newDrivekS != driveFeedforward.getKs() || newDrivekV != driveFeedforward.getKv() || newDrivekA != driveFeedforward.getKa())
            driveFeedforward = new SimpleMotorFeedforward(newDrivekS, newDrivekV, newDrivekA);

        // double newSteerkP = steerkPEntry.get();
        // if(newSteerkP != steerController.getP()) steerController.setP(newSteerkP);

        // double newSteerkI = steerkIEntry.get();
        // if(newSteerkI != steerController.getI()) steerController.setI(newSteerkI);

        // double newSteerkD = steerkDEntry.get();
        // if(newSteerkD != steerController.getD()) steerController.setD(newSteerkD);

    }
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveRelativeEncoder.getVelocity(), getModuleRotation());
    }

    public SwerveModuleState getDesiredState(){
        return desiredState;
       // return null;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveRelativeEncoder.getPosition(), getModuleRotation());
    }

    public void setDesiredState(SwerveModuleState inputSwerveState) {
        //SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getModuleRotation());
       // SwerveModuleState optimizedState = new SwerveModuleState();

       // optimizedState.optimize(getModuleRotation());


    //    SwerveModuleState optimizedState = new SwerveModuleState();
    //    optimizedState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    //    optimizedState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angularOffset));

    //    optimizedState.optimize(getModuleRotation());


       //driverController.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
       //steerController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);
       inputSwerveState.optimize(getModuleRotation());
       this.desiredState = inputSwerveState;

      // this.desiredState = desiredState;
       
      
    }

    



    @Override
    public void periodic() {
        final double driveOutput = driveController.calculate(driveRelativeEncoder.getVelocity(), desiredState.speedMetersPerSecond);
        final double driveFeedforward = this.driveFeedforward.calculate(desiredState.speedMetersPerSecond);
        //System.out.println(driveFeedforward);
        driveMotor.setVoltage(driveOutput + driveFeedforward);
        steerController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
    }

    // private void correctRelativeEncoder() {
    //     double delta = getAbsoluteModuleRotation().getRadians()-getModuleRotation().getRadians();
    //     if(delta > Math.PI)
    //         delta -= 2 * Math.PI;

    //     if(delta < -180)
    //         delta += 2 * Math.PI;

    //     relativeSteerAdjustment += delta * relativeSteerAdjustmentFactor;

    // }

    public Rotation2d getModuleRotation() {
        //return new Rotation2d(steerRelativeEncoder.getPosition() + relativeSteerAdjustment);
        return new Rotation2d(MathUtil.angleModulus(steerRelativeEncoder.getPosition() + angularOffset + relativeSteerAdjustment)); // Handled by 
    }

    public Rotation2d getAbsoluteModuleRotation() {
        return new Rotation2d(steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI);
        // return new Rotation2d(MathUtil.angleModulus(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition()) + steerOffset));
    }
    
    private double getDriveVelocity(){
        return driveRelativeEncoder.getVelocity();
        
    }

    @Override
    public void runForward(double voltage) {
        driveMotor.setVoltage(voltage);
        steerController.setReference(0, ControlType.kPosition);
    }

    @Override
    public void runRotation(double voltage) {
        driveMotor.set(0);
        steerMotor.setVoltage(voltage);
    }

}
