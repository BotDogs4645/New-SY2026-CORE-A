package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerVelocityRadPerSec = 0.0;
    public double rollerSupplyCurrent = 0.0;
    public double rollerAppliedVoltage = 0.0;
    public double armAngleRad = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double armSupplyCurrent = 0.0;
    public double armAppliedVoltage = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyOutputs(IntakeIOOutputs outputs) {}

  public default void setIntakeControl(ControlRequest control) {}

  enum IntakeOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP,
    POSITION
  }

  class IntakeIOOutputs {
    public IntakeOutputMode rollerMode = IntakeOutputMode.COAST;
    public double goalSpeedRadPerSec = 0.0;

    // Arm
    public IntakeOutputMode armMode = IntakeOutputMode.BRAKE;
    public double armGoalRadPosition = 0.0;
  }
}
