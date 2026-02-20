package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double shooterVelocityRadPerSec = 0.0;
    public double shooterSupplyCurrent = 0.0;
    public double shooterAppliedVoltage = 0.0;

    public double kickerVelocityRadPerSec = 0.0;
    public double kickerSupplyCurrent = 0.0;
    public double kickerAppliedVoltage = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterControl(ControlRequest control) {}

  public default void setKickerControl(ControlRequest control) {}

  public default void setAllControls(ControlRequest control) {
    setShooterControl(control);
    setKickerControl(control);
  }
}
