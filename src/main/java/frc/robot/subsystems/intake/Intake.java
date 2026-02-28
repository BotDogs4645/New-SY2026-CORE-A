package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import frc.robot.subsystems.intake.IntakeIO.IntakeOutputMode;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends FullSubsystem {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputs outputs = new IntakeIOOutputs();

  // Goals
  private double rollerGoalRadPerSec = 0.0;
  private double armGoalRadPosition = 0.0;
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
    } else {
      outputs.rollerMode = IntakeOutputMode.CLOSED_LOOP;
      outputs.goalSpeedRadPerSec = rollerGoalRadPerSec;
    }

    outputs.armMode = IntakeOutputMode.POSITION;
    outputs.armGoalRadPosition = armGoalRadPosition;
    io.applyOutputs(outputs);
  }

  public void setRollerSpeedRadPerSec(double speedRadPerSec) {
    rollerGoalRadPerSec = speedRadPerSec;
  }

  public void setArmGoalRadPosition(double armRadPosition) {
    armGoalRadPosition = armRadPosition;
  }

  public Command rollersInHeld() {
    return startEnd(
        () -> setRollerSpeedRadPerSec(IntakeConstants.kRollerInRadPerSec),
        () -> setRollerSpeedRadPerSec(0.0));
  }

  public Command armDown() {
    return runOnce(() -> setArmGoalRadPosition(IntakeConstants.kArmDownRad));
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          setRollerSpeedRadPerSec(0.0);
        });
  }
}
