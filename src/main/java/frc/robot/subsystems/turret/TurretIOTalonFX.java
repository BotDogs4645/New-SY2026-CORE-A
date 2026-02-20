package frc.robot.subsystems.turret;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor =
      new TalonFX(TurretConstants.motorCanId, TurretConstants.motorCanBus);

  private final StatusSignal<Current> supplyCurrent = turretMotor.getSupplyCurrent();
  private final StatusSignal<Angle> positionRot = turretMotor.getPosition();
  private final StatusSignal<Angle> velocityRotPerSec = turretMotor.getPosition();

  public TurretIOTalonFX() {
    var motorConfig = new TalonFXConfiguration();

    // motor config goes here

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
}
