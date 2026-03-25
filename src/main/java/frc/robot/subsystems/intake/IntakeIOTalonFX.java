package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX rollerMotor =
      new TalonFX(IntakeConstants.rollerMotorCanId, IntakeConstants.rollerMotorCanBus);
  private final TalonFX armMotor =
      new TalonFX(IntakeConstants.armMotorCanID, IntakeConstants.armMotorCanBus);

  private final StatusSignal<Current> rollerSupplyCurrent = rollerMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> rollerVelocityRotPerSec = rollerMotor.getVelocity();
  private final StatusSignal<Voltage> rollerVoltage = rollerMotor.getMotorVoltage();
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);
  private final DutyCycleOut armDutyCycleOut = new DutyCycleOut(0);
  private final NeutralOut neutralOut = new NeutralOut();
  private final CoastOut coastOut = new CoastOut();
  private final MotionMagicVoltage armRequest = new MotionMagicVoltage(0.0);
  private final StatusSignal<Current> armSupplyCurrent = armMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> armVelocityRotPerSec = armMotor.getVelocity();
  private final StatusSignal<Voltage> armVoltage = armMotor.getMotorVoltage();
  private final StatusSignal<Angle> armPositionRot = armMotor.getPosition();
  private final StatusSignal<Integer> faultField = armMotor.getFaultField();

  private final Debouncer armConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rollersConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    Logger.recordOutput("Intake/Roller/OutputMode", outputs.rollerMode.name());
    Logger.recordOutput("Intake/Roller/OutputLevel", outputs.rollerOutputLevel);
    switch (outputs.rollerMode) {
      case COAST -> {
        // make sure motor is in coast if you want it truly coasting
        rollerMotor.setControl(coastOut);
        break;
      }
      case BRAKE -> {
        rollerMotor.setControl(neutralOut);
        break;
      }
      case DUTY_CYCLE -> {
        rollerMotor.setControl(dutyCycleRequest.withOutput(outputs.rollerOutputLevel));
        break;
      }
    }
    Logger.recordOutput("Intake/Arm/OutputMode", outputs.armMode.name());
    Logger.recordOutput("Intake/Arm/OutputLevel", outputs.armOutputPower);
    Logger.recordOutput("Intake/Arm/TargetPositionRad", outputs.armGoalPosition.motorPositionRad);
    switch (outputs.armMode) {
      case POSITION -> {
        double rotations = Units.radiansToRotations(outputs.armGoalPosition.motorPositionRad);
        armMotor.setControl(armRequest.withPosition(rotations));
        break;
      }
      case COAST -> {
        armMotor.setControl(coastOut);
        break;
      }
      case BRAKE -> {
        armMotor.setControl(neutralOut);
        break;
      }
      case DUTY_CYCLE -> {
        armMotor.setControl(armDutyCycleOut.withOutput(outputs.armOutputPower));
        break;
      }
    }
  }

  public IntakeIOTalonFX() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted =
        IntakeConstants.rollersInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    // intake config here
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(intakeConfig, 0.25));

    var armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armConfig.Slot0.kP = IntakeConstants.armKP;
    armConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armConfig.Slot0.kG = IntakeConstants.armKG;
    armConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    armConfig.Voltage.PeakForwardVoltage = IntakeConstants.armPeakForwardVoltage;
    armConfig.Voltage.PeakReverseVoltage = IntakeConstants.armPeakReverseVoltage;

    armConfig.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.armMotionMagicCruiseVelocity;
    armConfig.MotionMagic.MotionMagicAcceleration = IntakeConstants.armMotionMagicAcceleration;
    tryUntilOk(5, () -> armMotor.getConfigurator().apply(armConfig, 0.25));

    armMotor.setPosition(IntakeConstants.kArmUpRad);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rollerSupplyCurrent,
        rollerVelocityRotPerSec,
        rollerVoltage,
        armSupplyCurrent,
        armVelocityRotPerSec,
        armVoltage,
        armPositionRot);
    ParentDevice.optimizeBusUtilizationForAll(rollerMotor, armMotor);
    Logger.recordOutput("Intake/Roller/talonFXInitialized", true);
    Logger.recordOutput("Intake/Arm/talonFXInitialized", true);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var rollerStatus =
        BaseStatusSignal.refreshAll(rollerSupplyCurrent, rollerVelocityRotPerSec, rollerVoltage);
    var armStatus =
        BaseStatusSignal.refreshAll(
            armSupplyCurrent, armVelocityRotPerSec, armVoltage, armPositionRot);

    inputs.rollerSupplyCurrent = rollerSupplyCurrent.getValueAsDouble();
    inputs.rollerVelocityRadPerSec =
        Units.rotationsToRadians(rollerVelocityRotPerSec.getValueAsDouble());
    inputs.rollerAppliedVoltage = rollerVoltage.getValueAsDouble();

    inputs.rollerConnected = rollersConnectedDebounce.calculate(rollerStatus.isOK());

    inputs.armSupplyCurrent = armSupplyCurrent.getValueAsDouble();
    inputs.armVelocityRadPerSec = Units.rotationsToRadians(armVelocityRotPerSec.getValueAsDouble());
    inputs.armAppliedVoltage = armVoltage.getValueAsDouble();
    inputs.armAngleRad = Units.rotationsToRadians(armPositionRot.getValueAsDouble());
    inputs.armConnected = armConnectedDebounce.calculate(armStatus.isOK());
  }

  @Override
  public void setIntakeControl(ControlRequest control) {
    rollerMotor.setControl(control);
  }

  @Override
  public void setArmEncoderPosition(double positionRad) {
    armMotor.setPosition(Units.radiansToRotations(positionRad));
  }
}
