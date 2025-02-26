package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final SparkMax leadMotor, followMotor;
  private final RelativeEncoder encoder;
  private SparkMaxConfig leadMotorConfig, followMotorConfig;

  // Constructor
  public ElevatorIOSparkMax() {
    // Initialize the CANSparkMax motors for main and follower
    leadMotor = new SparkMax(ElevatorConstants.kLeadMotorID, MotorType.kBrushless);
    followMotor = new SparkMax(ElevatorConstants.kFollowMotorID, MotorType.kBrushless);

    followMotorConfig.inverted(true);
    followMotorConfig.follow(leadMotor);

    // Initialize the encoder for main
    encoder = leadMotor.getEncoder();

    leadMotorConfig.closedLoop.p(ElevatorConstants.kElevatorP);
    leadMotorConfig.closedLoop.i(ElevatorConstants.kElevatorI);
    leadMotorConfig.closedLoop.d(ElevatorConstants.kElevatorD);
    leadMotorConfig.closedLoop.velocityFF(ElevatorConstants.kElevatorFF);
    leadMotorConfig.closedLoop.outputRange(ElevatorConstants.kElevatorMinOutput, ElevatorConstants.kElevatorMaxOutput);

    leadMotorConfig.idleMode(ElevatorConstants.kElevatorIdleMode);
    followMotorConfig.idleMode(ElevatorConstants.kElevatorIdleMode);

    leadMotor.configure(leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void set(double voltage) {
    // Set the power to the main motor
    leadMotor.set(voltage);
  }

  @Override
  public double getPosition() {
    // Get the position from the encoder
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    // Get the velocity from the encoder
    return encoder.getVelocity();
  }

  @Override
  public void resetPosition() {
    // Reset the encoder to the specified position
    encoder.setPosition(0);
  }

  @Override
  public void setPosition(double position) {
    leadMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  @Override
  public void stop() {
    leadMotor.setVoltage(0);
  }
}
