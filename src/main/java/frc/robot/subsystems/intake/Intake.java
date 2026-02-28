package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.subsystems.intake.IntakeIO.IntakeOutputMode;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends FullSubsystem {

  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputs outputs = new IntakeIOOutputs();

  // Goals
  private double rollerGoalRadPerSec = 0.0;

  // State helpers
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
      outputs.mode = IntakeOutputMode.COAST;
      outputs.goalSpeedRadPerSec = 0.0;
    } else {
      outputs.mode = IntakeOutputMode.CLOSED_LOOP;
      outputs.goalSpeedRadPerSec = rollerGoalRadPerSec;
    }
    io.applyOutputs(outputs);
  }

  public void setRollerSpeedRadPerSec(double speedRadPerSec) {
    rollerGoalRadPerSec = speedRadPerSec;
  }

  public Command rollersInHeld() {
    return startEnd(
        () -> setRollerSpeedRadPerSec(IntakeConstants.kRollerInRadPerSec),
        () -> setRollerSpeedRadPerSec(0.0));
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          setRollerSpeedRadPerSec(0.0);
        });
  }
}
