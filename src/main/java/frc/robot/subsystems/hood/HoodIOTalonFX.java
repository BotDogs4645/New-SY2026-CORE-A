package frc.robot.subsystems.hood;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor =
      new TalonFX(HoodConstants.motorCanId, HoodConstants.motorCanBus);
  private final CANcoder throughboreEncoder = new CANcoder(HoodConstants.thorughboreEncoderId);

  private final StatusSignal<Current> supplyCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Angle> positionRot = hoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = hoodMotor.getVelocity();

  private final MotionMagicDutyCycle motionMagicPositionCycle = new MotionMagicDutyCycle(0);
  private final NeutralOut brakeControl = new NeutralOut();

  public HoodIOTalonFX() {
    var motorConfig = new TalonFXConfiguration();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.motionMagicCruiseVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = HoodConstants.motionMagicAcceleration;
    motorConfig.Slot0.kP = 7;
    motorConfig.Feedback.FeedbackRemoteSensorID = HoodConstants.thorughboreEncoderId;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(motorConfig, 0.25));

    var encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    tryUntilOk(5, () -> throughboreEncoder.getConfigurator().apply(encoderConfig, 0.25));

    throughboreEncoder.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, supplyCurrent, positionRot, velocityRotPerSec);
    ParentDevice.optimizeBusUtilizationForAll(hoodMotor);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(supplyCurrent, positionRot, velocityRotPerSec);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.supplyCurrent = Units.rotationsToRadians(supplyCurrent.getValueAsDouble());
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    if (outputs.mode == HoodOutputMode.BRAKE) {
      hoodMotor.setControl(brakeControl);
    } else {
      hoodMotor.setControl(motionMagicPositionCycle.withPosition(outputs.targetPosition));
    }
  }
}
