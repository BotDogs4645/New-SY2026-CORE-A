package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public double supplyCurrent = 0.0;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public boolean connected = false;
  }

  enum SpindexerOutputMode {
    BRAKE,
    COAST,
    DUTY_CYCLE
  }

  public class SpindexerIOOutputs {
    SpindexerOutputMode mode = SpindexerOutputMode.BRAKE;
    double speed = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void setMotorControl(ControlRequest control) {}

  public default void applyOutputs(SpindexerIOOutputs outputs) {}
}
