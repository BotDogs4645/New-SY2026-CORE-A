package frc.robot.subsystems.turret;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.robot.subsystems.shooter.ShooterConstants;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor =
      new TalonFX(TurretConstants.motorCanId, TurretConstants.motorCanBus);

  private final StatusSignal<Current> supplyCurrent = turretMotor.getSupplyCurrent();
  private final StatusSignal<Angle> positionRot = turretMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = turretMotor.getVelocity();
  private final DutyCycleOut shooterActiveDutyCycle =
      new DutyCycleOut(ShooterConstants.shooterActiveVoltageProportion);
  private final DutyCycleOut kickerActiveDutyCycle =
      new DutyCycleOut(ShooterConstants.kickerActiveVoltageProportion);
  private final NeutralOut neutralControl = new NeutralOut();
  private final CoastOut coastControl = new CoastOut();

  public TurretIOTalonFX() {
    var motorConfig = new TalonFXConfiguration();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.motionMagicCruiseVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.motionMagicAcceleration;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.forwardLimit;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.reverseLimit;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    tryUntilOk(5, () -> turretMotor.getConfigurator().apply(motorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, supplyCurrent, positionRot, velocityRotPerSec);
    ParentDevice.optimizeBusUtilizationForAll(turretMotor);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(supplyCurrent, positionRot, velocityRotPerSec);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.supplyCurrent = Units.rotationsToRadians(supplyCurrent.getValueAsDouble());
  }

  @Override
  public void setMotorControl(ControlRequest control) {
    turretMotor.setControl(control);
  }

  // sets the position of the built in TalonFX encoder in radians
  @Override
  public void setEncoderPosition(double positionRad) {
    turretMotor.setPosition(Units.radiansToRotations(positionRad));
  }
}
