package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public double kickerVelocityRadPerSec = 0.0;
    public double kickerSupplyCurrent = 0.0;
    public double kickerAppliedVoltage = 0.0;
  }

  public default void updateInputs(KickerIOInputs inputs) {}

  public default void applyOutputs(KickerIOOutputs outputs) {}

  public default void setKickerControl(ControlRequest control) {}

  enum KickerOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP
  }

  class KickerIOOutputs {
    public KickerOutputMode kickerMode = KickerOutputMode.COAST;
    public double kickerGoalSpeedRadPerSec = 0.0;
  }
}
