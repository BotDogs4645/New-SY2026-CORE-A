package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerVelocityRadPerSec = 0.0;
    public double rollerSupplyCurrent = 0.0;
    public double rollerAppliedVoltage = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyOutputs(IntakeIOOutputs outputs) {}

  public default void setIntakeControl(ControlRequest control) {}

  enum IntakeOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP
  }

  class IntakeIOOutputs {
    public IntakeOutputMode mode = IntakeOutputMode.COAST;
    public double goalSpeedRadPerSec = 0.0;
  }
}
