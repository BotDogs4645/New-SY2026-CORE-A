package frc.robot.subsystems.spindexer;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class SpindexerIOTalonFX implements SpindexerIO {
  private final TalonFX spindexerMotor =
      new TalonFX(SpindexerConstants.motorCanId, SpindexerConstants.motorCanBus);

  private final StatusSignal<Current> supplyCurrent = spindexerMotor.getSupplyCurrent();
  private final StatusSignal<Angle> positionRot = spindexerMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = spindexerMotor.getVelocity();

  public SpindexerIOTalonFX() {
    var motorConfig = new TalonFXConfiguration();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        SpindexerConstants.motionMagicCruiseVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = SpindexerConstants.motionMagicAcceleration;

    tryUntilOk(5, () -> spindexerMotor.getConfigurator().apply(motorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, supplyCurrent, positionRot, velocityRotPerSec);
    ParentDevice.optimizeBusUtilizationForAll(spindexerMotor);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(supplyCurrent, positionRot, velocityRotPerSec);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.supplyCurrent = Units.rotationsToRadians(supplyCurrent.getValueAsDouble());
  }

  @Override
  public void setMotorControl(ControlRequest control) {
    spindexerMotor.setControl(control);
  }
}
