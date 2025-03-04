package frc.robot.subsystems.intake;

import java.util.EnumMap;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;


public class Intake extends SubsystemBase {

  public enum wristPositions{
    HOLD_ANGLE,
    L1_ANGLE,
    L2_ANGLE,
    L3_ANGLE,
    L4_ANGLE,
    SOURCE_ANGLE
  }

  public static EnumMap<wristPositions, Double> wristMap = new EnumMap<>(wristPositions.class);


  private final IntakeIO io;
  private final IntakeIO.IntakeIOInputsAutoLogged inputs = new IntakeIO.IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;


    wristMap.put(wristPositions.HOLD_ANGLE, WristConstants.HOLD_ANGLE);
    wristMap.put(wristPositions.L1_ANGLE, WristConstants.L1_ANGLE);
    wristMap.put(wristPositions.L2_ANGLE, WristConstants.L2_ANGLE);
    wristMap.put(wristPositions.L3_ANGLE, WristConstants.L3_ANGLE);
    wristMap.put(wristPositions.L4_ANGLE, WristConstants.L4_ANGLE);
    wristMap.put(wristPositions.SOURCE_ANGLE, WristConstants.SOURCE_ANGLE);
  }

  public void setAlgaeVoltage(double voltage) {
    io.setAlgaeVoltage(voltage);
  }

  public void setCoralIntakeVoltage(double voltage) {
    io.setCoralIntakeVoltage(voltage);
  }

  private double targetPosition = 0.0;
  private final ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.577, 0.0);

  public void setWristPositionDegrees(double position) {
    double targetPosition = Math.toRadians(position);
  }

  public double getTargetWristPosition() {
    return targetPosition;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public double getCoralWristIntakeCurrent() {
    return inputs.coralWristCurrent;
  }

  public double getWristPosition() {
    return inputs.coralWristPosition;
  }

  public void wristAngle(Intake.wristPositions position) {
    io.wristAngle(position);
  }

  public void setWristVoltage(double voltage) {
    System.out.println(getWristPosition());
    io.setWristVoltage(voltage);
  }

  public void resetAngle(double radians) {}
}