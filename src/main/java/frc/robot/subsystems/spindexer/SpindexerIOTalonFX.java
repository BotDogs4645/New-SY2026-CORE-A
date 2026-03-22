package frc.robot.subsystems.spindexer;

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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.Logger;

public class SpindexerIOTalonFX implements SpindexerIO {
  private final TalonFX spindexerMotor =
      new TalonFX(SpindexerConstants.motorCanId, SpindexerConstants.motorCanBus);

  private final StatusSignal<Current> supplyCurrent = spindexerMotor.getSupplyCurrent();
  private final StatusSignal<Angle> positionRot = spindexerMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = spindexerMotor.getVelocity();

  private final NeutralOut brakeControl = new NeutralOut();
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
  private final CoastOut coastControl = new CoastOut();

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public SpindexerIOTalonFX() {
    var motorConfig = new TalonFXConfiguration();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity =
        SpindexerConstants.motionMagicCruiseVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = SpindexerConstants.motionMagicAcceleration;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted =
        SpindexerConstants.isInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    tryUntilOk(5, () -> spindexerMotor.getConfigurator().apply(motorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, supplyCurrent, positionRot, velocityRotPerSec);
    ParentDevice.optimizeBusUtilizationForAll(spindexerMotor);
    Logger.recordOutput("Spindexer/talonFXInitialized", true);
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(supplyCurrent, positionRot, velocityRotPerSec);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.supplyCurrent = Units.rotationsToRadians(supplyCurrent.getValueAsDouble());
    inputs.connected = connectedDebounce.calculate(status.isOK());
  }

  @Override
  public void setMotorControl(ControlRequest control) {
    spindexerMotor.setControl(control);
  }

  @Override
  public void applyOutputs(SpindexerIOOutputs outputs) {
    Logger.recordOutput("Spindexer/OutputMode", outputs.mode.name());
    if (outputs.mode == SpindexerOutputMode.BRAKE) {
      Logger.recordOutput("Spindexer/OutputLevel", 0.0);
      setMotorControl(brakeControl);
    } else {
      Logger.recordOutput("Spindexer/OutputLevel", outputs.speed);
      setMotorControl(dutyCycleControl.withOutput(outputs.speed));
    }
  }
}
