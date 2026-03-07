package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.util.PhoenixUtil.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor = new TalonFX(TurretConstants.motorCanId, TurretConstants.motorCanBus);

  private final StatusSignal<Current> supplyCurrent = turretMotor.getSupplyCurrent();
  private final StatusSignal<Angle> positionRot = turretMotor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = turretMotor.getVelocity();

  private final NeutralOut brakeRequest = new NeutralOut();
  private final CoastOut coastRequest = new CoastOut();
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

  public TurretIOTalonFX() {
    var motorConfig = new TalonFXConfiguration();

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.motionMagicCruiseVelocity;
    motorConfig.MotionMagic.MotionMagicAcceleration = TurretConstants.motionMagicAcceleration;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.forwardLimit;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.reverseLimit;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfig.Slot0.kP = TurretConstants.kP;
    motorConfig.Slot0.kD = TurretConstants.kD;
    motorConfig.Slot0.kI = TurretConstants.kI;
    motorConfig.Slot0.kS = TurretConstants.kS;

    tryUntilOk(5, () -> turretMotor.getConfigurator().apply(motorConfig, 0.25));
    turretMotor.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, supplyCurrent, positionRot, velocityRotPerSec);
    ParentDevice.optimizeBusUtilizationForAll(turretMotor);
    turretMotor.setControl(brakeRequest);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    BaseStatusSignal.refreshAll(supplyCurrent, positionRot, velocityRotPerSec);

    inputs.positionRad = Units.rotationsToRadians(positionRot.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocityRotPerSec.getValueAsDouble());
    inputs.supplyCurrent = Units.rotationsToRadians(supplyCurrent.getValueAsDouble());
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE:
        turretMotor.setControl(brakeRequest);

      case COAST:
        turretMotor.setControl(coastRequest);

      case POSITION:
        double positionRot = convertToTurretPosition(outputs.goalPositionRad);
        Logger.recordOutput("Turret/Raw/goalPositionRot", positionRot);
        turretMotor.setControl(positionRequest.withPosition(positionRot));
    }
  }

  public double convertToTurretPosition(double angleRad) {
    Rotation2d withInitialPos = Rotation2d.fromRadians(angleRad).minus(new Rotation2d(TurretConstants.physicalStartingPosition));
    return withInitialPos.getRotations() * TurretConstants.gearRatio;
  }

}
