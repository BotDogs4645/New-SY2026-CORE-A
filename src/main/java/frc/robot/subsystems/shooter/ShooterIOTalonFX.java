package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterMotor =
      new TalonFX(ShooterConstants.shooterMotorCanId, ShooterConstants.shooterMotorCanBus);
  private final TalonFX kickerMotor =
      new TalonFX(ShooterConstants.kickerMotorCanId, ShooterConstants.kickerMotorCanBus);

  private final StatusSignal<Current> shooterSupplyCurrent = shooterMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> shooterVelocityRotPerSec = shooterMotor.getVelocity();
  private final StatusSignal<Voltage> shooterVoltage = shooterMotor.getMotorVoltage();

  private final StatusSignal<Current> kickerSupplyCurrent = kickerMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> kickerVelocityRotPerSec = kickerMotor.getVelocity();
  private final StatusSignal<Voltage> kickerVoltage = kickerMotor.getMotorVoltage();

  public ShooterIOTalonFX() {
    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.Voltage.PeakForwardVoltage = 1;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 10;
    tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(shooterConfig, 0.25));

    var kickerConfig = new TalonFXConfiguration();
    // shooter config here
    tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(kickerConfig, 0.25));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        shooterSupplyCurrent,
        shooterVelocityRotPerSec,
        shooterVoltage,
        kickerSupplyCurrent,
        kickerVelocityRotPerSec,
        kickerVoltage);
    ParentDevice.optimizeBusUtilizationForAll(shooterMotor, kickerMotor);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        shooterSupplyCurrent,
        shooterVelocityRotPerSec,
        shooterVoltage,
        kickerSupplyCurrent,
        kickerVelocityRotPerSec,
        kickerVoltage);

    inputs.shooterSupplyCurrent = shooterSupplyCurrent.getValueAsDouble();
    inputs.shooterVelocityRadPerSec =
        Units.rotationsToRadians(shooterVelocityRotPerSec.getValueAsDouble());
    inputs.shooterAppliedVoltage = shooterVoltage.getValueAsDouble();

    inputs.kickerSupplyCurrent = kickerSupplyCurrent.getValueAsDouble();
    inputs.kickerVelocityRadPerSec =
        Units.rotationsToRadians(kickerVelocityRotPerSec.getValueAsDouble());
    inputs.kickerAppliedVoltage = kickerVoltage.getValueAsDouble();
  }

  @Override
  public void setShooterControl(ControlRequest control) {
    shooterMotor.setControl(control);
  }

  @Override
  public void setKickerControl(ControlRequest control) {
    kickerMotor.setControl(control);
  }
}
