// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;
import  com.kauailabs.navx.frc.AHRS;

import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.KrakenMotorConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.revrobotics.RelativeEncoder;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import com.revrobotics.spark.config.SparkMaxConfig;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private YAGSLSubsystem m_frontLeft = new YAGSLSubsystem();

  private YAGSLSubsystem m_frontRight = new YAGSLSubsystem();

  private YAGSLSubsystem m_rearLeft = new YAGSLSubsystem();

  private YAGSLSubsystem m_rearRight = new YAGSLSubsystem();

  // The gyro sensor
  private final AHRS m_gyro = new AHRS();

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter; //= new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter; //= new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private final boolean isTuning = false;

  private double transKp = 1.75;
  private double transKi = 0.1;
  private double transKd = 0.1;

  private double rotKp = 1.5;
  private double rotKi = 0.0;
  private double rotKd = 0.0;


  //m_frontLeft.getState(),
 //                                                          m_frontRight.getState(),
 //                                                          m_rearLeft.getState(),
 //                                                          m_rearRight.getState()

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. 
  public DriveSubsystem() {

    m_magLimiter = new SlewRateLimiter(DriveConstants.kAutoMagnitudeSlewRate);
    m_rotLimiter = new SlewRateLimiter(DriveConstants.kAutoDirctionalSlewRate);

    m_frontLeft.SwerveModule( 
          KrakenMotorConstants.kFrontLeftDrivingCanId, 
          NeoMotorConstants.kFrontLeftTurningCanId,
          NeoMotorConstants.kFrontLeftCanCoderCanId);
    m_frontRight.SwerveModule( 
          KrakenMotorConstants.kFrontRightDrivingCanId, 
          NeoMotorConstants.kFrontRightTurningCanId,
          NeoMotorConstants.kFrontRightCanCoderCanId);
    m_rearLeft.SwerveModule( 
          KrakenMotorConstants.kRearLeftDrivingCanId, 
          NeoMotorConstants.kRearLeftTurningCanId,
          KrakenMotorConstants.kRearLeftCanCoderCanId);
    m_rearRight.SwerveModule( 
          KrakenMotorConstants.kRearRightDrivingCanId, 
          NeoMotorConstants.kRearRightTurningCanId,
          KrakenMotorConstants.kRearRightCanCoderCanId);
    //SmartDashboard.putNumber("Auto/Drive/transKp", transKp);
    //SmartDashboard.putNumber("Auto/Drive/transKi", transKi);
    //SmartDashboard.putNumber("Auto/Drive/transKd", transKd);

    //SmartDashboard.putNumber("Auto/Drive/rotKp", rotKp);
    //SmartDashboard.putNumber("Auto/Drive/rotKi", rotKi);
    //SmartDashboard.putNumber("Auto/Drive/rotKd", rotKd);

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
      //m_gyro.calibrate();
      AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
              new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                      //new PIDConstants(1.5, .1, 0.1), // Translation PID constants 3, .01, .1
                      new PIDConstants(transKp, .1, .1),//1.75p for 4 note
                      new PIDConstants(1.5, 0.0, 0.0) // Rotation PID constants
                      //4.8, // Max module speed
                      //0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                      //new ReplanningConfig() // Default path replanning config. See the API for the options here
              ),
              config,
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
  
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
    } catch (Exception e){
      e.printStackTrace();
    }

  }

  StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();

  StructArrayPublisher<Pose2d> pub = NetworkTableInstance.getDefault().getStructArrayTopic("Pose", Pose2d.struct).publish();

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

    SmartDashboard.putNumber("Gyro Current Angle", -m_gyro.getAngle());
    //SmartDashboard.putNumber("Pitch", getPitch());
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

        SwerveModuleState[] states = new SwerveModuleState[] {
    m_frontLeft.getState(),
    m_frontRight.getState(),
    m_rearLeft.getState(),
    m_rearRight.getState()
    };

    Pose2d [] poses = new Pose2d[] {
      getPose()
    };

    //SmartDashboard.putNumber("FrontLeft", m_frontLeft.getAppliedOutput());
    //SmartDashboard.putNumber("FrontRight", m_frontRight.getAppliedOutput());
    //SmartDashboard.putNumber("RearLeft", m_rearLeft.getAppliedOutput());
        publisher.set(states);
        pub.set(poses);
    if(isTuning)
    {
      tuneNumbers();
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

    public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d(), getPositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
                                                           m_frontRight.getState(),
                                                           m_rearLeft.getState(),
                                                           m_rearRight.getState());
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false,false);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   
   * @param desiredStates The desired SwerveModule states.
   
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0.
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. 
  public void zeroHeading() {
    m_gyro.reset();
  }

  public void zeroHaw() {
    m_gyro.zeroYaw();
    
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  public void setToCoast() {
    m_frontLeft.setToCoast();
    m_frontRight.setToCoast();
    m_rearLeft.setToCoast();
    m_rearRight.setToCoast();
  }

  public void tuneNumbers()
  {
    boolean changedConfig = false;
    if(transKp != SmartDashboard.getNumber("Auto/Drive/transKp", transKp))
    {
      transKp = SmartDashboard.getNumber("Auto/Drive/transKp", transKp);
      changedConfig = true;
    }
    if(transKi != SmartDashboard.getNumber("Auto/Drive/transKi", transKi))
    {
      transKi = SmartDashboard.getNumber("Auto/Drive/transKi", transKi);
      changedConfig = true;
    }
    if(transKd != SmartDashboard.getNumber("Auto/Drive/transKd", transKd))
    {
      transKd = SmartDashboard.getNumber("Auto/Drive/transKd", transKd);
      changedConfig = true;
    }

    if(rotKp != SmartDashboard.getNumber("Auto/Drive/rotKp", rotKp))
    {
      rotKp = SmartDashboard.getNumber("Auto/Drive/rotKp", rotKp);
      changedConfig = true;
    }if(rotKi != SmartDashboard.getNumber("Auto/Drive/rotKi", rotKi))
    {
      rotKi = SmartDashboard.getNumber("Auto/Drive/rotKi", rotKi);
      changedConfig = true;
    }
    if(rotKd != SmartDashboard.getNumber("Auto/Drive/rotKd", rotKd))
    {
      rotKd = SmartDashboard.getNumber("Auto/Drive/rotKd", rotKd);
      changedConfig = true;
    }
    if(changedConfig == true)
    {
      RobotConfig config;
      try{
        config = RobotConfig.fromGUISettings();
        AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
              new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                      //new PIDConstants(1.5, .1, 0.1), // Translation PID constants 3, .01, .1
                      new PIDConstants(transKp, transKi, transKd),
                      new PIDConstants(rotKp, rotKi, rotKd) // Rotation PID constants
                      //4.8, // Max module speed
                      //0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                      //new ReplanningConfig() // Default path replanning config. See the API for the options here
              ),
              config,
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE O RIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
    } catch (Exception e){
      SmartDashboard.putBoolean("yest", true);
    }
  }
}

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void changeTeleopSlew()
  {
    m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    m_rotLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularSpeed);
  }

  public double getPitch() { return m_gyro.getPitch(); }
  public double getRoll() { return m_gyro.getRoll(); }
  public double getYaw() { return m_gyro.getYaw(); }
}*/