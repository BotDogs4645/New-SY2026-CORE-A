package frc.robot.subsystems.shooter;

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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX shooterMotor =
      new TalonFX(ShooterConstants.motorCanId, ShooterConstants.motorCanBus);
  private final StatusSignal<Current> shooterSupplyCurrent = shooterMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> shooterVelocityRotPerSec = shooterMotor.getVelocity();
  private final StatusSignal<Voltage> shooterVoltage = shooterMotor.getMotorVoltage();
  private final CoastOut coastRequest = new CoastOut();
  private final NeutralOut brakeRequest = new NeutralOut();
  private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0.0);

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ShooterIOTalonFX() {
    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted =
        ShooterConstants.motorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.Slot0.kV = ShooterConstants.kV;
    shooterConfig.Slot0.kP = ShooterConstants.kP;
    tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(shooterConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, shooterSupplyCurrent, shooterVelocityRotPerSec, shooterVoltage);
    ParentDevice.optimizeBusUtilizationForAll(shooterMotor);
    Logger.recordOutput("Shooter/talonFXInitialized", true);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(shooterSupplyCurrent, shooterVelocityRotPerSec, shooterVoltage);

    inputs.shooterSupplyCurrent = shooterSupplyCurrent.getValueAsDouble();
    inputs.shooterVelocityRadPerSec =
        Units.rotationsToRadians(shooterVelocityRotPerSec.getValueAsDouble());
    inputs.shooterAppliedVoltage = shooterVoltage.getValueAsDouble();
    inputs.connected = connectedDebounce.calculate(status.isOK());
  }

  @Override
  public void setShooterControl(ControlRequest control) {
    shooterMotor.setControl(control);
  }

  @Override
  public void applyOutputs(ShooterIOOutputs outputs) {
    Logger.recordOutput("Shooter/OutputMode", outputs.shooterMode.name());
    switch (outputs.shooterMode) {
      case BRAKE:
        Logger.recordOutput("Shooter/GoalSpeedRadPerSec", 0.0);
        shooterMotor.setControl(brakeRequest);

      case COAST:
        Logger.recordOutput("Shooter/GoalSpeedRadPerSec", 0.0);
        shooterMotor.setControl(coastRequest);

      case CLOSED_LOOP:
        Logger.recordOutput("Shooter/GoalSpeedRadPerSec", outputs.shooterGoalSpeedRadPerSec);
        double targetRotPerSec = Units.radiansToRotations(outputs.shooterGoalSpeedRadPerSec);
        shooterMotor.setControl(shooterVelocityRequest.withVelocity(targetRotPerSec));
    }
  }
}
