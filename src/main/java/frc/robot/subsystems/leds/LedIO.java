package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {

  @AutoLog
  public static class LedIOInputs {
    public double currentAmps = 0.0;
    public double temp = 0.0;
    public int fault = 0;
  }

  public default void updateInputs(LedIOInputs inputs) {}
}
