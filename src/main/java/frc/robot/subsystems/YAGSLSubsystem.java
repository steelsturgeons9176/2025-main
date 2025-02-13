// package frc.robot.subsystems;

// import com.ctre.phoenix6.hardware.CANcoder;
// import com.ctre.phoenix6.signals.SensorDirectionValue;
// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.CANcoderConfigurator;
// import com.ctre.phoenix6.configs.MagnetSensorConfigs;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ModuleConstants;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



// public class YAGSLSubsystem {
//     private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
//     private SparkMax driveMotor;
//     private SparkMax steerMotor;
//     private CANcoder absoluteEncoder;
//     private SparkClosedLoopController drivingPIDController;
//     private SparkClosedLoopController turningPIDController;
//     private RelativeEncoder driveEncoder;
//     private RelativeEncoder steerEncoder;
//     private SparkMaxConfig configSteer;
//     private SparkMaxConfig configDrive;
//     private double chassisAngularOffset = 0;

//     public void SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID)
//     {
//         driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
//         steerMotor = new SparkMax(steerMotorCANID, MotorType.kBrushless);
//         absoluteEncoder = new CANcoder(cancoderCANID);
//         configSteer = new SparkMaxConfig();
//         configDrive = new SparkMaxConfig();
        
//         drivingPIDController = driveMotor.getClosedLoopController();
//         turningPIDController = steerMotor.getClosedLoopController();
        
      
//         driveEncoder = driveMotor.getEncoder();
//         steerEncoder = steerMotor.getEncoder();
        
       
//         absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
        
       
//         CANcoderConfigurator cfg = absoluteEncoder.getConfigurator();
//         cfg.apply(new CANcoderConfiguration());
//         MagnetSensorConfigs  magnetSensorConfiguration = new MagnetSensorConfigs();
//         cfg.refresh(magnetSensorConfiguration);
//         cfg.apply(magnetSensorConfiguration
    
//                   .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

      
//         configSteer.inverted(false);
//         configSteer.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    
//         configSteer.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
//         configSteer.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
        
//         configSteer.closedLoop.positionWrappingEnabled(true);
//         configSteer.closedLoop.positionWrappingMinInput(0);
//         configSteer.closedLoop.positionWrappingMaxInput(90);
        
       
//         configSteer.closedLoop.p(ModuleConstants.kTurningP);
//         configSteer.closedLoop.i(ModuleConstants.kTurningI);
//         configSteer.closedLoop.d(ModuleConstants.kTurningD);
//         configSteer.closedLoop.velocityFF(ModuleConstants.kTurningFF);
       
//         configDrive.inverted(false);
     
//         configDrive.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
             
//         configDrive.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);        
//         configDrive.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
        
//         configDrive.closedLoop.p(ModuleConstants.kDrivingP);
//         configDrive.closedLoop.i(ModuleConstants.kDrivingI);
//         configDrive.closedLoop.d(ModuleConstants.kDrivingD);
//         configDrive.closedLoop.velocityFF(ModuleConstants.kDrivingFF); 

//         driveMotor.configure(configDrive, ResetMode.kResetSafeParameters,  PersistMode.kPersistParameters);
//         steerMotor.configure(configSteer, ResetMode.kResetSafeParameters,  PersistMode.kPersistParameters);
//         driveEncoder.setPosition(0);
        
//         steerEncoder.setPosition(absoluteEncoder.getAbsolutePosition(true).getValueAsDouble() * 360);
//     }
    
    
//     /**
//     Get the distance in meters.
//     */
//     public double getDistance()
//     {
//         return driveEncoder.getPosition();
//     }
    
//     /**
//     Get the angle.
//     */
//     public Rotation2d getAngle()
//     {
//           return Rotation2d.fromDegrees(steerEncoder.getPosition());
//     }
    
//     /**
//     Set the swerve module state.
//     @param state The swerve module state to set.
//     */
//     public void setState(SwerveModuleState state)
//     {
//           turningPIDController.setReference(state.angle.getDegrees(), ControlType.kPosition);
//           drivingPIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
//     }

    

//     public SwerveModulePosition getPosition() {
//     // Apply chassis angular offset to the encoder position to get the position
//     // relative to the chassis.
//     return new SwerveModulePosition(
//         driveEncoder.getPosition(),
//         new Rotation2d(steerEncoder.getPosition() - chassisAngularOffset));
//   }

//     public SwerveModuleState getState() {
//         // Apply chassis angular offset to the encoder position to get the position
//         // relative to the chassis.
//         return new SwerveModuleState(driveEncoder.getVelocity(),
//             new Rotation2d(steerEncoder.getPosition() - chassisAngularOffset));
//       }
    
//       public void setDesiredState(SwerveModuleState desiredState) {
//         // Apply chassis angular offset to the desired state.
//         SwerveModuleState correctedDesiredState = new SwerveModuleState();
//         correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
//         correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
    
//         // Optimize the reference state to avoid spinning further than 90 degrees.
//         SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
//             new Rotation2d(steerEncoder.getPosition()));
    
//         // Command driving and turning SPARKS MAX towards their respective setpoints.
//         drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
//         turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
    
//         m_desiredState = desiredState;
//       }  

//       public void resetEncoders() {
//         driveEncoder.setPosition(0);
//       }

//     public void setToCoast()
//   {
//     configDrive.idleMode(IdleMode.kCoast);
//     steerMotor.configure(configSteer, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//   }
// }