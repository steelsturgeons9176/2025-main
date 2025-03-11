// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.07399577167019;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    //public static final double kDirectionSlewRate = 1.2; // radians per second
    //public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    //public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    public static final double kDirectionSlewRate = 3.8; // radians per second
    public static final double kAutoMagnitudeSlewRate = 1.8;
    public static final double kAutoDirctionalSlewRate = 1.2;
    public static final double kMagnitudeSlewRate = 15.6; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 12.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;


    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = .75;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 15; // amps


  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;
  }

  public static final class NeoMotorConstants {

    public static final int kFrontLeftTurningCanId = 21;
    public static final int kRearLeftTurningCanId = 51;
    public static final int kRearRightTurningCanId = 41;
    public static final int kFrontRightTurningCanId = 31;
    public static final int kFrontLeftCanCoderCanId = 0;
    public static final int kFrontRightCanCoderCanId = 0;
    public static final int kFreeSpeedRpm = 1;
  }
 // will add once we have our krakens positioned
  public static final class KrakenMotorConstants {
    public static final int kFrontLeftDrivingCanId = 21;
    public static final int kFrontRightDrivingCanId = 0;
    public static final int kRearLeftDrivingCanId = 51;
    public static final int kRearRightDrivingCanId = 41;
    public static final int kRearLeftCanCoderCanId = 0;
    public static final int kRearRightCanCoderCanId = 0;
    public static final int KEndifactorCanId = 0; //14 after arm is taken off
    public static final int kCANRangecCanId = 0; //15 once wrist deleted
  }

  public static final class GlobalConstants {
    public static final double loopPeriodSecs = 0.02;
  }

// Changed ArmConstants to ClimbArmConstants for when we get the arm on the elevator

  public static final class ClimbArmConstants {
    public static final int ARM_MOTOR_ID = 6;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
    public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
    public static final double ARM_SPEED_DOWN = 0.4;
    public static final double ARM_SPEED_UP = -0.4;
    public static final double ARM_HOLD_DOWN = 0.1;
    public static final double ARM_HOLD_UP = -0.15;
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 7;
    public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 60;
    public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
    public static final double CLIMBER_SPEED_DOWN = -0.5;
    public static final double CLIMBER_SPEED_UP = 0.5;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double TURN_CONSTANT    = 6;
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
  }

  public static class WristConstants {
    public static final double WRIST_CONVERSION_FACTOR = 15;
    public static final double HOLD_ANGLE = -.2; // TODO:
    public static final double L1_ANGLE = 0;
    public static final double L2_ANGLE = -.1;
    public static final double L3_ANGLE = -.1;
    public static final double L4_ANGLE = -.3;
    public static final double SOURCE_ANGLE = -.15; // TODO:
    public static final double K_WRIST_MIN_OUTPUT = -.15;
    public static final double K_WRIST_MAX_OUTPUT = .25;
    public static final int k_Wrist_Max_CURRENT_LIMIT = 30;
    public static final double K_WRIST_STARTING_OFFSET = .2;
  }


  public static final class ArmConstants {
    public static final int ARM_MOTOR_ID = 6;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
    public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
    public static final double ARM_SPEED_DOWN = 0.4;
    public static final double ARM_SPEED_UP = -0.4;
    public static final double ARM_HOLD_DOWN = 0.1;
    public static final double ARM_HOLD_UP = -0.15;

    public static final double kOffset = 0;
    public static final double kMinHeightAbs = 0 + kOffset;
    public static final double kMaxHeightAbs = .25 + kOffset;
    public static final double STOWED = (10.0/360.0f) + kOffset; // 0 degrees ~ 0/360 - 0
    public static final double INTAKE = 0.0472 + kOffset; // 17 degrees ~ 17/360 - 0.0472
    public static final double SOURCE = .125 + kOffset; // 45 degrees ~ 90/360 - 0.125
    public static final double SUBSHOT = (53.0f/360.0f) + kOffset; // 60 degrees ~ 60/360 - 0.166
    public static final double PODSHOT = (33.0f/360.0f) + kOffset; //TODO: revert back to 33.0f
    public static final double LONGSHOT = (26.25f/360.0f) + kOffset;
    public static final double AMP = .25 + kOffset; // 90 degrees ~ 90/360 - 0.25
    public static final double POOP = (7.0f/360.0f) + kOffset;
    public static final double kArmMinOutput = -.6;
    public static final double kArmMaxOutput = .6;
    public static final double kAllowedErrAbs = .008f;

    public static final int ARM_ALGAE_MOTOR1_ID = 12;
    public static final int ARM_ALGAE_MOTOR2_ID = 13;
    public static final int ARM_CORAL_MOTOR_ID = 14;
    public static final int ARM_WRIST_MOTOR_ID = 15;
  }


  public static final class EndifactorConstant{
    public final static int KLeadNeoId = 17; 
    public final static int KFallowNeoId = 18; 




  }
  public static final class ElevatorConstants {
    public static final int kLeadMotorID = 10;
    public static final int kFollowMotorID = 11;
    public static final double kElevatorP = 0.027;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.0;
    public static final double kElevatorFF = 0.0085;
    public static final double kElevatorMinOutput = 0;
    public static final double kElevatorMaxOutput = 70;
    public static final IdleMode kElevatorIdleMode = IdleMode.kBrake;
    public static final double L1 = 0.125;
    public static final double L2 = 6.75;
    public static final double L3 = 12.5;
    public static final double L4 = 19;
    public static final double SOURCE = 0;
  }
}