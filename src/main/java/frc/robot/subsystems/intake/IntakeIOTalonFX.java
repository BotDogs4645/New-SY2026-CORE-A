package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX rollerMotor =
      new TalonFX(IntakeConstants.rollerMotorCanId, IntakeConstants.rollerMotorCanBus);
  private final StatusSignal<Current> rollerSupplyCurrent = rollerMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> rollerVelocityRotPerSec = rollerMotor.getVelocity();
  private final StatusSignal<Voltage> rollerVoltage = rollerMotor.getMotorVoltage();
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final NeutralOut neutralOut = new NeutralOut();

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST -> {
        // make sure motor is in coast if you want it truly coasting
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        rollerMotor.setControl(neutralOut);
      }
      case BRAKE -> {
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.setControl(neutralOut);
      }
      case CLOSED_LOOP -> {
        // Convert rad/s -> rotations/s for Phoenix velocity control
        double rps = Units.radiansToRotations(outputs.goalSpeedRadPerSec);
        rollerMotor.setControl(velocityRequest.withVelocity(rps));
      }
    }
  }

  public IntakeIOTalonFX() {
    var intakeConfig = new TalonFXConfiguration();
    // intake config here
    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(intakeConfig, 0.25));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, rollerSupplyCurrent, rollerVelocityRotPerSec, rollerVoltage);
    ParentDevice.optimizeBusUtilizationForAll(rollerMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(rollerSupplyCurrent, rollerVelocityRotPerSec, rollerVoltage);

    inputs.rollerSupplyCurrent = rollerSupplyCurrent.getValueAsDouble();
    inputs.rollerVelocityRadPerSec =
        Units.rotationsToRadians(rollerVelocityRotPerSec.getValueAsDouble());
    inputs.rollerAppliedVoltage = rollerVoltage.getValueAsDouble();
  }

  @Override
  public void setIntakeControl(ControlRequest control) {
    rollerMotor.setControl(control);
  }
}
