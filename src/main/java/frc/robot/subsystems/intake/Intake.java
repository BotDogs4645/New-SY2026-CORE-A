package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.subsystems.intake.IntakeIO.IntakeOutputMode;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends FullSubsystem {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputs outputs = new IntakeIOOutputs();

  // Goals
  private double rollerGoalRadPerSec = 0.0;
  private double armGoalRadPosition = 0.0;
  private IntakeOutputMode armOutputMode = IntakeOutputMode.COAST;
  // State helpers but its lowkenuinely dead code rn
  private boolean rollerAtGoal = false;

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
    if (rollerGoalRadPerSec == 0.0) {
      outputs.rollerMode = IntakeOutputMode.COAST;
      outputs.goalSpeedRadPerSec = 0.0;
      Logger.recordOutput("Intake/outputMode", "COAST");
    } else {
      outputs.rollerMode = IntakeOutputMode.DUTY_CYCLE;
      outputs.goalSpeedRadPerSec = rollerGoalRadPerSec;
      Logger.recordOutput("Intake/outputMode", "DUTYCYCLE");
      Logger.recordOutput("Intake/rollerOutput", rollerGoalRadPerSec);
    }
    outputs.armMode = armOutputMode;
    outputs.armGoalRadPosition = armGoalRadPosition;
    io.applyOutputs(outputs);
  }

  public void setRollerSpeedRadPerSec(double speedRadPerSec) {
    rollerGoalRadPerSec = speedRadPerSec;
  }

  public void setArmGoalRadPosition(double armRadPosition) {
    armOutputMode = IntakeOutputMode.POSITION;
    armGoalRadPosition = armRadPosition;
  }

  @AutoLogOutput
  public boolean armAtGoal() {
    return Math.abs(inputs.armAngleRad - armGoalRadPosition) < 0.6;
  }

  public Command rollersInHeld() {
    return startEnd(
        () -> setRollerSpeedRadPerSec(IntakeConstants.kRollerInRadPerSec),
        () -> setRollerSpeedRadPerSec(0.0));
  }

  public Command armDown() {
    return Commands.sequence(
        Commands.runOnce(() -> setArmGoalRadPosition(IntakeConstants.kArmDownRadHalf), this),
        Commands.waitUntil(this::armAtGoal),
        Commands.runOnce(() -> setRollerSpeedRadPerSec(0.35), this),
        Commands.runOnce(() -> setArmGoalRadPosition(IntakeConstants.kArmDownRad), this),
        Commands.waitUntil(this::armAtGoal),
        Commands.runOnce(
            () -> {
              setRollerSpeedRadPerSec(0);
              armOutputMode = IntakeOutputMode.COAST;
            },
            this));
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          setRollerSpeedRadPerSec(0.0);
        });
  }

  public Command startIntake() {
    return runEnd(
        () -> {
          setRollerSpeedRadPerSec(0.42);
        },
        () -> {
          setRollerSpeedRadPerSec(0);
        });
  }
}
