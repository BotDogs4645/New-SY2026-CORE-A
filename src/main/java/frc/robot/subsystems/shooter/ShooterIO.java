package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterVelocityRadPerSec = 0.0;
    public double shooterSupplyCurrent = 0.0;
    public double shooterAppliedVoltage = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void applyOutputs(ShooterIOOutputs outputs) {}

  public default void setShooterControl(ControlRequest control) {}

  enum ShooterOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP
  }

  class ShooterIOOutputs {
    public ShooterOutputMode mode = ShooterOutputMode.COAST;
    public double goalSpeedRadPerSec = 0.0;
  }
}
