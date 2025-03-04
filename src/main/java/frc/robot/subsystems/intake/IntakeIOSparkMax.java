package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;


// import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOSparkMax implements IntakeIO {
  SparkMax algaeMotor1;
  SparkMax algaeMotor2;
  SparkMax coralIntake;
  SparkMax coralWrist;

  SparkMaxConfig algeaMotorConfig;
  SparkMaxConfig coralIntakeConfig;
  SparkMaxConfig coralWristConfig;

  RelativeEncoder wristEncoder;

  SparkClosedLoopController m_pidController;

  public IntakeIOSparkMax() {
    // find actual motor IDs
    algaeMotor1 = new SparkMax(ArmConstants.ARM_ALGAE_MOTOR1_ID, MotorType.kBrushless);
    algaeMotor2 = new SparkMax(ArmConstants.ARM_ALGAE_MOTOR2_ID, MotorType.kBrushless);
    coralIntake = new SparkMax(ArmConstants.ARM_CORAL_MOTOR_ID, MotorType.kBrushless);
    coralWrist = new SparkMax(ArmConstants.ARM_WRIST_MOTOR_ID, MotorType.kBrushless);

    algeaMotorConfig = new SparkMaxConfig();
    coralIntakeConfig = new SparkMaxConfig();
    coralWristConfig = new SparkMaxConfig();

    m_pidController = coralWrist.getClosedLoopController();


    algeaMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(15);


    coralIntakeConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(15);


    coralWristConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40);

    coralWristConfig.closedLoop
      .p(0.55)
      .i(0)
      .d(0.0)
      .velocityFF(0.00375)
      .outputRange(WristConstants.K_WRIST_MIN_OUTPUT, WristConstants.K_WRIST_MAX_OUTPUT);

    // ask about gear ratios for all motors
    wristEncoder = coralWrist.getEncoder();

    algaeMotor1.configure(algeaMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    algeaMotorConfig.inverted(true);
    algaeMotor2.configure(algeaMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralIntake.configure(coralIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralWrist.configure(coralWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.coralWristCurrent = coralWrist.getOutputCurrent();
    inputs.coralWristVelocity = coralWrist.getEncoder().getVelocity();
    inputs.coralWristPosition = coralWrist.getEncoder().getPosition();
  }

  @Override
  public void setAlgaeVoltage(double voltage) {
    algaeMotor1.setVoltage(voltage);
    algaeMotor2.setVoltage(voltage);
    // instead of changing it to inverted we can change the voltage to negative
  }

  @Override
  public void setCoralIntakeVoltage(double voltage) {
    coralIntake.setVoltage(voltage);
  }

  @Override
  public void adjustAngle(double angleRadians) {
    coralWrist.getEncoder().setPosition(coralWrist.getEncoder().getPosition() + angleRadians);
  }

  @Override
  public void wristAngle(Intake.wristPositions position) {
    // System.out.println("Wrist position: " + getWristPosition());
    double ref = Intake.wristMap.get(position);
    m_pidController.setReference(ref, SparkMax.ControlType.kPosition);
  }

  @Override
  public double getWristPosition() {
    return wristEncoder.getPosition();
  }

  @Override
  public void setWristVoltage(double voltage) {
    // System.out.println("Wrist position: " + getWristPosition());
    coralWrist.set(voltage);
  }
}
