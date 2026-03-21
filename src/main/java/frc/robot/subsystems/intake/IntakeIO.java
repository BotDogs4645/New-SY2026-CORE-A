package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.subsystems.intake.IntakeConstants.ArmMechanismPosition;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerVelocityRadPerSec = 0.0;
    public double rollerSupplyCurrent = 0.0;
    public double rollerAppliedVoltage = 0.0;
    public boolean rollerConnected = false;

    public double armAngleRad = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double armSupplyCurrent = 0.0;
    public double armAppliedVoltage = 0.0;
    public boolean armConnected = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyOutputs(IntakeIOOutputs outputs) {}

  public default void setIntakeControl(ControlRequest control) {}

  public default void setArmEncoderPosition(double positionRad) {}

  enum IntakeOutputMode {
    BRAKE,
    COAST,
    CLOSED_LOOP,
    DUTY_CYCLE,
    POSITION
  }

  class IntakeIOOutputs {
    public IntakeOutputMode rollerMode = IntakeOutputMode.COAST;
    public double rollerOutputLevel = 0.0;

    // Arm
    public IntakeOutputMode armMode = IntakeOutputMode.BRAKE;
    public ArmMechanismPosition armGoalPosition = ArmMechanismPosition.ARM_UP;
    public double armOutputPower = 0.0;
  }
}
