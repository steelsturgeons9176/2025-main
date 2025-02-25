package frc.robot.subsystems.intake;

// import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {

  // @AutoLog
  public static class IntakeIOInputs {
    public double coralWristCurrent = 0.0;
    public double coralWristVelocity = 0.0;
    public double coralWristPosition = 0.0;
  }


  class IntakeIOInputsAutoLogged extends IntakeIOInputs implements LoggableInputs {
    @Override
    public void toLog(LogTable table) {
      table.put("coralWristCurrent", coralWristCurrent);
      table.put("coralWristVelocity", coralWristVelocity);
      table.put("coralWristPosition", coralWristPosition);
    }

    @Override
    public void fromLog(LogTable table) {
      coralWristCurrent = table.get("coralWristCurrent", coralWristCurrent);
      coralWristVelocity = table.get("coralWristVelocity", coralWristVelocity);
      coralWristPosition = table.get("coralWristPosition", coralWristPosition);
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setAlgaeVoltage(double voltage) {}

  public default void setCoralIntakeVoltage(double voltage) {}

  public default void setCoralWristPosition(double position, double ffvoltage) {}

  public default void adjustAngle(double angleRadians) {}

  public default void wristAngle(double position) {}

  public default double getWristPosition() {
    return 0;
  }

  public default void setWristVoltage(double voltage) {}
}
