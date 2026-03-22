package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double supplyCurrent = 0.0;
    public double positionRad = 0.0;
    public double positionRadWithGearRatio = 0.0;
    public double velocityRadPerSec = 0.0;
    public String controlMode = "";
    public boolean connected;
  }

  enum TurretOutputMode {
    POSITION,
    BRAKE,
    COAST
  }

  class TurretIOOutputs {
    public TurretOutputMode mode = TurretOutputMode.BRAKE;
    public double goalPositionRad = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void applyOutputs(TurretIOOutputs outputs) {}

  public default void setEncoderPosition(double positionRad) {}
}
