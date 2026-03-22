// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.KickerIO.KickerIOOutputs;
import frc.robot.subsystems.kicker.KickerIO.KickerOutputMode;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Kicker extends FullSubsystem {

  private KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
  private final KickerIOOutputs outputs = new KickerIOOutputs();
  public Alert kickerDisconnectedAlert =
      new Alert("IO Status", "Kicker disconnected!", AlertType.kError);

  private double kickerGoalSpeedRadPerSec = 0.0;
  private boolean atGoalSpeed = false;

  /** Creates a new Kicker. */
  public Kicker(KickerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
    kickerDisconnectedAlert.set(!inputs.connected);
  }

  @AutoLogOutput
  public boolean isAtGoalSpeed() {
    return atGoalSpeed;
  }

  @Override
  public void periodicAfterScheduler() {
    if (kickerGoalSpeedRadPerSec == 0.0) {
      outputs.kickerMode = KickerOutputMode.BRAKE;
      outputs.kickerGoalSpeedRadPerSec = 0.0;
      atGoalSpeed = false;
    } else {
      outputs.kickerMode = KickerOutputMode.CLOSED_LOOP;
      outputs.kickerGoalSpeedRadPerSec = kickerGoalSpeedRadPerSec;
    }
    atGoalSpeed = Math.abs(inputs.kickerVelocityRadPerSec - kickerGoalSpeedRadPerSec) < 10;

    io.applyOutputs(outputs);
  }

  public void setKickerGoalSpeedRadPerSec(double speedRadPerSec) {
    kickerGoalSpeedRadPerSec = speedRadPerSec;
  }

  public Command StopKicker() {
    return runOnce(
        () -> {
          setKickerGoalSpeedRadPerSec(0.0);
        });
  }

  public Command RunKicker() {
    return startEnd(
        () -> setKickerGoalSpeedRadPerSec(KickerConstants.defaultSpeedRadPerSec),
        () -> setKickerGoalSpeedRadPerSec(0));
  }

  public Command StartKicker() {
    return runOnce(
        () -> {
          setKickerGoalSpeedRadPerSec(KickerConstants.defaultSpeedRadPerSec);
        });
  }
}
