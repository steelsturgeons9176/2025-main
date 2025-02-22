/*package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.kinematics.SwerveModulePosition;


public class YAGSLSubsystem {

    private SparkMax driveMotor;
    private SparkMax steerMotor;
    private CANcoder absoluteEncoder;
    private SparkClosedLoopController drivingPIDController;
    private SparkClosedLoopController turningPIDController;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder steerEncoder;
    private SparkMaxConfig configSteer;
    private SparkMaxConfig configDrive;
    private RelativeEncoder m_drivingEncoder;
    private RelativeEncoder m_turningEncoder;
    private CANcoder m_absoluteEncoder;
    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private SparkClosedLoopController m_drivingPIDController;
    private SparkClosedLoopController m_turningPIDController;
    private final SparkMaxConfig m_drivingSparkMaxConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_turningSparkMaxConfig = new SparkMaxConfig();
    private SparkMax m_drivingSparkMax;
    private SparkMax m_turningSparkMax;

    public void SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID)
    {
        driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerMotorCANID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(cancoderCANID);
        configSteer = new SparkMaxConfig();
        configDrive = new SparkMaxConfig();
        
        // Get the PID Controllers
        drivingPIDController = driveMotor.getClosedLoopController();
        turningPIDController = steerMotor.getClosedLoopController();
        
        // Get the encoders
        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();
        
        // Reset everything to factory default
       // driveMotor.configure(configDrive, null,  PersistMode.kPersistParameters);
       // steerMotor.configure(configSteer, null,  PersistMode.kPersistParameters);
        absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
        
        // Continue configuration here..
        
        // CANcoder Configuration
        CANcoderConfigurator cfg = absoluteEncoder.getConfigurator();
        cfg.apply(new CANcoderConfiguration());
        MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
        cfg.refresh(magnetSensorConfiguration);
        cfg.apply(magnetSensorConfiguration
      //            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        // Steering Motor Configuration
        configSteer.inverted(false);
        configSteer.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        // Apply position and velocity conversion factors for the turning encoder. We
        // want these in radians and radians per second to use with WPILib's swerve
        // APIs.
        configSteer.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        configSteer.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        configSteer.closedLoop.positionWrappingEnabled(true);
        configSteer.closedLoop.positionWrappingMinInput(0);
        configSteer.closedLoop.positionWrappingMaxInput(90);
        
        // Set the PID gains for the turning motor. Note these are example gains, and you
        // may need to tune them for your own robot!
       // turningPIDController.setP(ModuleConstants.kTurningP);
      //  turningPIDController.setI(ModuleConstants.kTurningI);
      //  turningPIDController.setD(ModuleConstants.kTurningD);
       // turningPIDController.setFF(ModuleConstants.kTurningFF);
        //configSteer.closedLoop.pidf(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD, 
        //ModuleConstants.kTurningFF);
        configSteer.closedLoop.p(ModuleConstants.kTurningP);
        configSteer.closedLoop.i(ModuleConstants.kTurningI);
        configSteer.closedLoop.d(ModuleConstants.kTurningD);
        configSteer.closedLoop.velocityFF(ModuleConstants.kTurningFF);
        // Drive Motor Configuration
        configDrive.inverted(false);
       // drivingPIDController.setFeedbackDevice(driveEncoder);
        configDrive.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        // Apply position and velocity conversion factors for the driving encoder. The
        // native units for position and velocity are rotations and RPM, respectively,
        // but we want meters and meters per second to use with WPILib's swerve APIs.        
        configDrive.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);        
        configDrive.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        // Set the PID gains for the driving motor. Note these are example gains, and you
        // may need to tune them for your own robot!
        configDrive.closedLoop.p(ModuleConstants.kDrivingP);
        configDrive.closedLoop.i(ModuleConstants.kDrivingI);
        configDrive.closedLoop.d(ModuleConstants.kDrivingD);
        configDrive.closedLoop.velocityFF(ModuleConstants.kDrivingFF);
        
        // Save the SPARK MAX configurations. If a SPARK MAX browns out during
        // operation, it will maintain the above configurations.
        driveMotor.configure(configDrive, ResetMode.kResetSafeParameters,  PersistMode.kPersistParameters);
        steerMotor.configure(configSteer, ResetMode.kResetSafeParameters,  PersistMode.kPersistParameters);
          
        driveEncoder.setPosition(0);
        //steerEncoder.setPosition(absoluteEncoder.getAbsolutePosition().refresh().getValue() * 360);
        steerEncoder.setPosition(absoluteEncoder.getAbsolutePosition(true).getValueAsDouble() * 360);
    }
    
    
    /**
    Get the distance in meters.
    
    public double getDistance()
    {
        return driveEncoder.getPosition();
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(steerEncoder.getPosition()));
    }
    
    public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_absoluteEncoder.getPosition().getValueAsDouble() - m_chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        // Optimize the reference state to avoid spinning further than 90 degrees.
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
            new Rotation2d(m_absoluteEncoder.getPosition().getValueAsDouble()));

        // Command driving and turning SPARKS MAX towards their respective setpoints.
        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

        m_desiredState = desiredState;
    }

    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
        m_turningEncoder.setPosition(0);
    }

    public void setToCoast()
    {
        m_drivingSparkMaxConfig.idleMode(IdleMode.kCoast);
        m_drivingSparkMax.configure(null, ResetMode.kResetSafeParameters, null);
    }

    /**
    Get the angle.
    
    public Rotation2d getAngle()
    {
          return Rotation2d.fromDegrees(steerEncoder.getPosition());
    }
    
    /**
    Set the swerve module state.
    @param state The swerve module state to set.
    
    public void setState(SwerveModuleState state)
    {
          turningPIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
          drivingPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    }

}*/