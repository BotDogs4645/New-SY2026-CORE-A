package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.intake.IntakeConstants.ArmMechanismPosition;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.subsystems.intake.IntakeIO.IntakeOutputMode;
import frc.robot.util.FullSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends FullSubsystem {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputs outputs = new IntakeIOOutputs();

  // Goals
  private double rollerOutputLevel = 0.0;
  private ArmMechanismPosition armGoalPosition = ArmMechanismPosition.ARM_UP;
  private double armOutputPower = 0.0;
  private IntakeOutputMode armOutputMode = IntakeOutputMode.COAST;
  // State helpers but its lowkenuinely dead code rn
  private boolean rollerAtGoal = false;
  private boolean hasExtendedIntake = false;

  public Alert rollersDisconnectedAlert =
      new Alert("IO Status", "Intake rollers disconnected!", AlertType.kError);
  public Alert armDisconnectedAlert =
      new Alert("IO Status", "Intake arm disconnected!", AlertType.kError);

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    rollersDisconnectedAlert.set(!inputs.rollerConnected);
    armDisconnectedAlert.set(!inputs.armConnected);
  }

  @Override
  public void periodicAfterScheduler() {
    if (rollerOutputLevel == 0.0) {
      outputs.rollerMode = IntakeOutputMode.COAST;
      outputs.rollerOutputLevel = 0.0;
    } else {
      outputs.rollerMode = IntakeOutputMode.DUTY_CYCLE;
      outputs.rollerOutputLevel = rollerOutputLevel;
    }
    outputs.armMode = armOutputMode;
    outputs.armGoalPosition = armGoalPosition;
    outputs.armOutputPower = armOutputPower;
    io.applyOutputs(outputs);
  }

  public void setRollerOutput(double outputLevel) {
    rollerOutputLevel = outputLevel;
  }

  public void setArmGoalPosition(ArmMechanismPosition armGoalPosition) {
    armOutputMode = IntakeOutputMode.POSITION;
    this.armGoalPosition = armGoalPosition;
  }

  public void setArmDutyCycleOut(double power) {
    armOutputMode = IntakeOutputMode.DUTY_CYCLE;
    this.armOutputPower = power;
  }

  @AutoLogOutput(key = "Intake/Arm/IsAtGoalPosition")
  public boolean armAtGoalPosition() {
    return Math.abs(inputs.armAngleRad - armGoalPosition.motorPositionRad) < 1
        && armOutputMode == IntakeOutputMode.POSITION;
  }

  @AutoLogOutput(key = "Intake/Arm/IsStalled")
  public boolean isArmStalled() {
    return inputs.armSupplyCurrent > IntakeConstants.forceExtendArmStallCurrent
        && armOutputMode == IntakeOutputMode.DUTY_CYCLE;
  }
  
  @AutoLogOutput(key = "Intake/Roller/IsStalled")
  public boolean isRollersStalled() {
    return inputs.rollerSupplyCurrent > IntakeConstants.rollerStallCurrent
        && rollerOutputLevel != 0.0 && Math.abs(inputs.rollerVelocityRadPerSec) < IntakeConstants.rollerStallVelocityThreshold;
  }

  @AutoLogOutput
  public boolean hasExtendedIntake() {
    return hasExtendedIntake;
  }

  public Command RollersInHeld() {
    return startEnd(
        () -> setRollerOutput(IntakeConstants.kRollerInRadPerSec), () -> setRollerOutput(0.0));
  }

  public Command ExtendIntake() {

    return Commands.sequence(
            runOnce(() -> setArmGoalPosition(ArmMechanismPosition.ARM_HALF_DOWN)),
            Commands.waitUntil(this::armAtGoalPosition),
            runOnce(() -> setRollerOutput(IntakeConstants.armDownRollerOutput)),
            runOnce(() -> setArmDutyCycleOut(IntakeConstants.forceExtendArmOutput)),
            Commands.waitUntil(this::isArmStalled),
            runOnce(
                () -> {
                  io.setArmEncoderPosition(ArmMechanismPosition.ARM_DOWN.motorPositionRad);
                  setRollerOutput(0);
                  armOutputMode = IntakeOutputMode.COAST;
                  hasExtendedIntake = true;
                }))
        .unless(this::hasExtendedIntake);
  }

  public Command StopIntake() {
    return runOnce(
        () -> {
          setRollerOutput(0.0);
        });
  }

  public Command StartIntake() {
    return runOnce(
        () -> {
          setRollerOutput(IntakeConstants.intakingRollerOutput);
        });
  }

  public Command RunIntake(BooleanSupplier dislodgeBalls) {
    return Commands.repeatingSequence(
            runOnce(() -> {
              setRollerOutput(IntakeConstants.intakingRollerOutput);
              armOutputMode = IntakeOutputMode.COAST;
            }),
            Commands.waitUntil(this::isRollersStalled),
            RunOuttake(() -> false).withTimeout(0.5)
    ).finallyDo(() -> setRollerOutput(0));
  }

  public Command RunOuttake(BooleanSupplier dislodgeBalls) {
    return runEnd(
        () -> {
          setRollerOutput(-IntakeConstants.intakingRollerOutput);
          armOutputMode = IntakeOutputMode.COAST;
        },
        () -> {
          setRollerOutput(0);
        });
  }
}
