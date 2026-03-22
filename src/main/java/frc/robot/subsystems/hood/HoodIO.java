package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public double supplyCurrent = 0.0;
    public double positionRad = 0.0;
    public double positionRadWithoutOffset = 0.0;
    public double velocityRadPerSec = 0.0;
    public double startingOffsetRad = 0;
    public boolean encoderConnected = false;
    public boolean connected = false;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  enum HoodOutputMode {
    BRAKE,
    COAST,
    POSITION
  }

  class HoodIOOutputs {
    public HoodOutputMode mode = HoodOutputMode.BRAKE;
    public double targetPosition = 0.0;
  }

  public default void applyOutputs(HoodIOOutputs outputs) {}
}
