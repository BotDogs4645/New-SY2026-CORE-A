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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

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
  private final CoastOut coastRequest = new CoastOut();
  private final NeutralOut brakeRequest = new NeutralOut();
  private final VelocityVoltage shooterVelocityRequest = new VelocityVoltage(0.0);
  private final VelocityVoltage kickerVelocityRequest = new VelocityVoltage(0.0);

  public ShooterIOTalonFX() {
    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shooterConfig.Slot0.kV = 0.118;
    shooterConfig.Slot0.kP = 0.08;
    tryUntilOk(5, () -> shooterMotor.getConfigurator().apply(shooterConfig, 0.25));

    var kickerConfig = new TalonFXConfiguration();
    kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.Slot0.kV = 0.118;
    kickerConfig.Slot0.kP = 0.05;
    kickerConfig.Feedback.FeedbackRemoteSensorID = 19;
    tryUntilOk(5, () -> kickerMotor.getConfigurator().apply(kickerConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        shooterSupplyCurrent,
        shooterVelocityRotPerSec,
        shooterVoltage,
        kickerSupplyCurrent,
        kickerVelocityRotPerSec,
        kickerVoltage);
    ParentDevice.optimizeBusUtilizationForAll(shooterMotor);
    Logger.recordOutput("Shooter/talonFXInitialized", true);
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
  public void applyOutputs(ShooterIOOutputs outputs) {
    if (outputs.shooterMode == ShooterOutputMode.BRAKE) {
      shooterMotor.setControl(brakeRequest);
    } else {
      double targetRotPerSec = Units.radiansToRotations(outputs.shooterGoalSpeedRadPerSec);
      shooterMotor.setControl(shooterVelocityRequest.withVelocity(targetRotPerSec));
    }

    if (outputs.kickerMode == ShooterOutputMode.BRAKE) {
      kickerMotor.setControl(brakeRequest);
    } else {
      double targetRotPerSec = Units.radiansToRotations(outputs.kickerGoalSpeedRadPerSec);
      kickerMotor.setControl(shooterVelocityRequest.withVelocity(targetRotPerSec));
    }
  }
}
