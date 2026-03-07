package frc.robot.subsystems.kicker;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class KickerIOTalonFX implements KickerIO {
  private final TalonFX kickerMotor =
      new TalonFX(KickerConstants.motorCanId, KickerConstants.motorCanBus);
  private final StatusSignal<Current> kickerSupplyCurrent = kickerMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> kickerVelocityRotPerSec = kickerMotor.getVelocity();
  private final StatusSignal<Voltage> kickerVoltage = kickerMotor.getMotorVoltage();
  private final CoastOut coastRequest = new CoastOut();
  private final NeutralOut brakeRequest = new NeutralOut();
  private final VelocityVoltage kickerVelocityRequest = new VelocityVoltage(0.0);

  public KickerIOTalonFX() {
    var kickerConfig = new TalonFXConfiguration();
    kickerConfig.MotorOutput.Inverted = KickerConstants.motorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.Slot0.kV = KickerConstants.kV;
    kickerConfig.Slot0.kP = KickerConstants.kP;
    tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(kickerConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        kickerSupplyCurrent,
        kickerVelocityRotPerSec,
        kickerVoltage);
    ParentDevice.optimizeBusUtilizationForAll(kickerMotor);
    Logger.recordOutput("Kicker/talonFXInitialized", true);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        kickerSupplyCurrent,
        kickerVelocityRotPerSec,
        kickerVoltage);

    inputs.kickerSupplyCurrent = kickerSupplyCurrent.getValueAsDouble();
    inputs.kickerVelocityRadPerSec =
        Units.rotationsToRadians(kickerVelocityRotPerSec.getValueAsDouble());
    inputs.kickerAppliedVoltage = kickerVoltage.getValueAsDouble();
  }

  @Override
  public void setKickerControl(ControlRequest control) {
    kickerMotor.setControl(control);
  }

  @Override
  public void applyOutputs(KickerIOOutputs outputs) {
    switch(outputs.kickerMode) {
      case BRAKE:
        kickerMotor.setControl(brakeRequest);

      case COAST:
        kickerMotor.setControl(coastRequest);

      case CLOSED_LOOP:
        double targetRotPerSec = Units.radiansToRotations(outputs.kickerGoalSpeedRadPerSec);
        kickerMotor.setControl(kickerVelocityRequest.withVelocity(targetRotPerSec));
    }
  }
}
