package frc.robot.subsystems.hood;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.Logger;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor =
      new TalonFX(HoodConstants.motorCanId, HoodConstants.motorCanBus);
  private final CANcoder throughboreEncoder =
      new CANcoder(HoodConstants.thorughboreEncoderId, HoodConstants.encoderCanBus);

  private final StatusSignal<Current> supplyCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Angle> positionRot = hoodMotor.getPosition();
  private final StatusSignal<Angle> encoderPositionRot = throughboreEncoder.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = hoodMotor.getVelocity();

  private final MotionMagicDutyCycle motionMagicPositionCycle = new MotionMagicDutyCycle(0);
  private final StaticBrake brakeControl = new StaticBrake();

  // in rotations
  private double startingOffset = 0;

  private final Debouncer connectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public HoodIOTalonFX() {

    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset = 0.5;
    tryUntilOk(5, () -> throughboreEncoder.getConfigurator().apply(encoderConfig, 0.25));

    startingOffset = throughboreEncoder.getPosition().getValueAsDouble();

    var motorConfig = new TalonFXConfiguration();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.motionMagicCruiseVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = HoodConstants.motionMagicAcceleration;
    motorConfig.Slot0.kP = HoodConstants.kP;
    motorConfig.Slot0.kI = HoodConstants.kI;
    motorConfig.Slot0.kD = HoodConstants.kD;
    motorConfig.Slot0.kG = HoodConstants.kG;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfig.Feedback.FeedbackRemoteSensorID = HoodConstants.thorughboreEncoderId;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        HoodConstants.fowardSoftLimit + startingOffset;
    Logger.recordOutput("Hood/forwardSoftLimit", HoodConstants.fowardSoftLimit + startingOffset);
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        HoodConstants.reverseSoftLimit + startingOffset;
    Logger.recordOutput("Hood/reverseSoftLimit", HoodConstants.reverseSoftLimit + startingOffset);

    tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(motorConfig, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, supplyCurrent, positionRot, velocityRotPerSec);
    ParentDevice.optimizeBusUtilizationForAll(hoodMotor);
    Logger.recordOutput("Hood/talonFXInitialized", true);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(supplyCurrent, positionRot, velocityRotPerSec);

    inputs.connected = connectedDebounce.calculate(status.isOK());

    var encoderStatus = BaseStatusSignal.refreshAll(encoderPositionRot);
    inputs.encoderConnected = encoderStatus.isOK();

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.positionRadWithoutOffset =
        Units.rotationsToRadians(positionRot.getValueAsDouble() - startingOffset);
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.supplyCurrent = Units.rotationsToRadians(supplyCurrent.getValueAsDouble());
    inputs.startingOffsetRad = Units.rotationsToRadians(startingOffset);
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    Logger.recordOutput("Hood/OutputMode", outputs.mode.name());
    Logger.recordOutput("Hood/TargetPositionRad", outputs.targetPosition);
    Logger.recordOutput(
        "Hood/TargetPositionWithOffsetRad",
        Units.rotationsToRadians(
            Units.radiansToRotations(outputs.targetPosition) + startingOffset));
    if (outputs.mode == HoodOutputMode.BRAKE) {
      hoodMotor.setControl(brakeControl);
    } else {
      double positionRot = Units.radiansToRotations(outputs.targetPosition) + startingOffset;
      hoodMotor.setControl(motionMagicPositionCycle.withPosition(positionRot));
    }
  }
}
