package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX rollerMotor =
      new TalonFX(IntakeConstants.rollerMotorCanId, IntakeConstants.rollerMotorCanBus);
  private final TalonFX armMotor =
      new TalonFX(IntakeConstants.armMotorCanID, IntakeConstants.armMotorCanBus);

  private final StatusSignal<Current> rollerSupplyCurrent = rollerMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> rollerVelocityRotPerSec = rollerMotor.getVelocity();
  private final StatusSignal<Voltage> rollerVoltage = rollerMotor.getMotorVoltage();
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final NeutralOut neutralOut = new NeutralOut();
  private final MotionMagicVoltage armRequest = new MotionMagicVoltage(0.0);
  private final StatusSignal<Current> armSupplyCurrent = armMotor.getSupplyCurrent();
  private final StatusSignal<AngularVelocity> armVelocityRotPerSec = armMotor.getVelocity();
  private final StatusSignal<Voltage> armVoltage = armMotor.getMotorVoltage();
  private final StatusSignal<Angle> armPositionRot = armMotor.getPosition();

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    switch (outputs.rollerMode) {
      case COAST -> {
        // make sure motor is in coast if you want it truly coasting
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        rollerMotor.setControl(neutralOut);
      }
      case BRAKE -> {
        rollerMotor.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.setControl(neutralOut);
      }
      case CLOSED_LOOP -> {
        double rps = Units.radiansToRotations(outputs.goalSpeedRadPerSec);
        rollerMotor.setControl(velocityRequest.withVelocity(rps));
      }
      case POSITION -> {
        rollerMotor.setControl(neutralOut);
      }
    }
    switch (outputs.armMode) {
      case POSITION -> {
        double rotations = Units.radiansToRotations(outputs.armGoalRadPosition);
        armMotor.setControl(armRequest.withPosition(rotations));
      }
      case COAST -> {
        armMotor.setNeutralMode(NeutralModeValue.Coast);
        armMotor.setControl(neutralOut);
      }
      case BRAKE -> {
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotor.setControl(neutralOut);
      }
      case CLOSED_LOOP -> {
        armMotor.setControl(neutralOut);
      }
    }
  }

  public IntakeIOTalonFX() {
    var intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // intake config here
    tryUntilOk(5, () -> rollerMotor.getConfigurator().apply(intakeConfig, 0.25));

    var armConfig = new TalonFXConfiguration();
    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armConfig.Slot0.kP = 20.0;

    armConfig.Voltage.PeakForwardVoltage = 3.0;
    armConfig.Voltage.PeakReverseVoltage = -3.0;

    // REALLY slow (units are rotations/sec and rotations/sec^2)
    armConfig.MotionMagic.MotionMagicCruiseVelocity = 0.25; // very slow
    armConfig.MotionMagic.MotionMagicAcceleration = 0.50; // gentle accel
    tryUntilOk(5, () -> armMotor.getConfigurator().apply(armConfig, 0.25));

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
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rollerSupplyCurrent,
        rollerVelocityRotPerSec,
        rollerVoltage,
        armSupplyCurrent,
        armVelocityRotPerSec,
        armVoltage,
        armPositionRot);

    inputs.rollerSupplyCurrent = rollerSupplyCurrent.getValueAsDouble();
    inputs.rollerVelocityRadPerSec =
        Units.rotationsToRadians(rollerVelocityRotPerSec.getValueAsDouble());
    inputs.rollerAppliedVoltage = rollerVoltage.getValueAsDouble();

    inputs.armSupplyCurrent = armSupplyCurrent.getValueAsDouble();
    inputs.armVelocityRadPerSec = Units.rotationsToRadians(armVelocityRotPerSec.getValueAsDouble());
    inputs.armAppliedVoltage = armVoltage.getValueAsDouble();
    inputs.armAngleRad = Units.rotationsToRadians(armPositionRot.getValueAsDouble());
  }

  @Override
  public void setIntakeControl(ControlRequest control) {
    rollerMotor.setControl(control);
  }
}
