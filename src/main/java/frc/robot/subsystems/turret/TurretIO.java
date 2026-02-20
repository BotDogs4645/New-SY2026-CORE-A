package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double supplyCurrent = 0.0;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setMotorControl(ControlRequest control) {}
}
