package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private IntakeOutputMode armOutputMode = IntakeOutputMode.COAST;
  // State helpers but its lowkenuinely dead code rn
  private boolean rollerAtGoal = false;

  private boolean hasExtendedIntake = false;

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  @Override
  public void periodicAfterScheduler() {
    if (rollerOutputLevel == 0.0) {
      outputs.rollerMode = IntakeOutputMode.COAST;
      outputs.rollerOutputLevel = 0.0;
      Logger.recordOutput("Intake/outputMode", "COAST");
    } else {
      outputs.rollerMode = IntakeOutputMode.DUTY_CYCLE;
      outputs.rollerOutputLevel = rollerOutputLevel;
      Logger.recordOutput("Intake/outputMode", "DUTYCYCLE");
      Logger.recordOutput("Intake/rollerOutput", rollerOutputLevel);
    }
    outputs.armMode = armOutputMode;
    outputs.armGoalPosition = armGoalPosition;
    io.applyOutputs(outputs);
  }

  public void setRollerOutput(double outputLevel) {
    rollerOutputLevel = outputLevel;
  }

  public void setArmGoalPosition(ArmMechanismPosition armGoalPosition) {
    armOutputMode = IntakeOutputMode.POSITION;
    this.armGoalPosition = armGoalPosition;
  }

  @AutoLogOutput
  public boolean armAtGoal() {
    return Math.abs(inputs.armAngleRad - armGoalPosition.motorPositionRad) < 1;
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
            Commands.waitUntil(this::armAtGoal),
            runOnce(() -> setRollerOutput(IntakeConstants.armDownRollerOutput)),
            runOnce(() -> setArmGoalPosition(ArmMechanismPosition.ARM_DOWN)),
            Commands.waitUntil(this::armAtGoal),
            runOnce(
                () -> {
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
    return runEnd(
        () -> {
          setRollerOutput(IntakeConstants.intakingRollerOutput);
          if (dislodgeBalls.getAsBoolean()) {
            setArmGoalPosition(ArmMechanismPosition.DISLODGE_BALLS);
          } else {
            setArmGoalPosition(ArmMechanismPosition.ARM_DOWN);
          }
        },
        () -> {
          setRollerOutput(0);
        });
  }

  public Command RunOuttake(BooleanSupplier dislodgeBalls) {
    return runEnd(
        () -> {
          setRollerOutput(-IntakeConstants.intakingRollerOutput);
          if (dislodgeBalls.getAsBoolean()) {
            setArmGoalPosition(ArmMechanismPosition.DISLODGE_BALLS);
          } else {
            setArmGoalPosition(ArmMechanismPosition.ARM_DOWN);
          }
        },
        () -> {
          setRollerOutput(0);
        });
  }
}
