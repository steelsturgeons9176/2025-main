/*// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ModuleConstants;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class MAXSwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final SparkMaxConfig m_drivingSparkMaxConfig = new SparkMaxConfig();
  private final SparkMaxConfig m_turningSparkMaxConfig = new SparkMaxConfig();

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;
  private final CANcoder m_absoluteEncoder;

  private final SparkClosedLoopController m_drivingPIDController;
  private final SparkClosedLoopController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  
  private SparkMaxConfig configSteer;
  private SparkMaxConfig configDrive;
  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    configSteer = new SparkMaxConfig();
    configDrive = new SparkMaxConfig();

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.configure(configDrive, null,  PersistMode.kPersistParameters);
    m_turningSparkMax.configure(configSteer, null,  PersistMode.kPersistParameters);
    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getEncoder();
    m_absoluteEncoder = new CANcoder(m_turningSparkMax.getDeviceId());
    m_drivingPIDController = m_drivingSparkMax.getClosedLoopController();
    m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    configDrive.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    configSteer.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    configDrive.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);        
    configDrive.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    configSteer.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    configSteer.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    configSteer.inverted(ModuleConstants.kTurningEncoderInverted);
    
    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    configSteer.closedLoop.positionWrappingEnabled(true);
    configSteer.closedLoop.positionWrappingMinInput(0);
    configSteer.closedLoop.positionWrappingMaxInput(90);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingSparkMaxConfig.closedLoop.p(ModuleConstants.kDrivingP);
    m_drivingSparkMaxConfig.closedLoop.i(ModuleConstants.kDrivingI);
    m_drivingSparkMaxConfig.closedLoop.d(ModuleConstants.kDrivingD);
    m_drivingSparkMaxConfig.closedLoop.velocityFF(ModuleConstants.kDrivingFF);
    m_drivingSparkMaxConfig.closedLoop.outputRange(ModuleConstants.kDrivingMinOutput,
    ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    configSteer.closedLoop.p(ModuleConstants.kTurningP);
    configSteer.closedLoop.i(ModuleConstants.kTurningI);
    configSteer.closedLoop.d(ModuleConstants.kTurningD);
    configSteer.closedLoop.velocityFF(ModuleConstants.kTurningFF);
    configSteer.closedLoop.outputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);
        //setOutputRange
    m_drivingSparkMaxConfig.idleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMaxConfig.idleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkMaxConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMaxConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.configure(null, ResetMode.kResetSafeParameters, null);
    m_turningSparkMax.configure(null, ResetMode.kResetSafeParameters, null);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_absoluteEncoder.getPosition().getValueAsDouble());
    m_drivingEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_absoluteEncoder.getPosition().getValueAsDouble() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_absoluteEncoder.getPosition().getValueAsDouble() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState oldOptimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_absoluteEncoder.getPosition().getValueAsDouble()));

    SwerveModuleState optimizedDesiredState = new SwerveModuleState(
                                              correctedDesiredState.speedMetersPerSecond, 
                                              correctedDesiredState.angle);

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders.
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }

  public double getAppliedOutput()
  {
    return m_drivingSparkMax.getAppliedOutput();
  }

  public void setToCoast()
  {
    m_drivingSparkMaxConfig.idleMode(IdleMode.kCoast);
    m_drivingSparkMax.configure(null, ResetMode.kResetSafeParameters, null);
  }
}*/