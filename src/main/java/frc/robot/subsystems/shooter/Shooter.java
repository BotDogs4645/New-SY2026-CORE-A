// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOOutputs;
import frc.robot.subsystems.shooter.ShooterIO.ShooterOutputMode;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends FullSubsystem {

  private ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOOutputs outputs = new ShooterIOOutputs();
  public Alert shooterDisconnectedAlert =
      new Alert("IO Status", "Shooter disconnected!", AlertType.kError);

  private double shooterGoalSpeedRadPerSec = 0.0;
  private boolean atGoalSpeed = false;
  private boolean prevAtGoalSpeed = false;
  private int prevShotCounter = 0;
  private int shotCounter = 0;
  private Debouncer isAtGoalSpeedDebouncer = new Debouncer(0.06, DebounceType.kFalling);

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    shooterDisconnectedAlert.set(!inputs.connected);
  }

  @AutoLogOutput
  public boolean isAtGoalSpeed() {
    if (prevAtGoalSpeed && !atGoalSpeed) {
      shotCounter++;
    }
    prevAtGoalSpeed = atGoalSpeed;
    return atGoalSpeed;
  }

  @Override
  public void periodicAfterScheduler() {
    if (shooterGoalSpeedRadPerSec == 0.0) {
      outputs.shooterMode = ShooterOutputMode.BRAKE;
      outputs.shooterGoalSpeedRadPerSec = 0.0;
      // atGoalSpeed =
      // isAtGoalSpeedDebouncer.calculate(
      // Math.abs(shooterGoalSpeedRadPerSec - inputs.shooterVelocityRadPerSec) < 13);
      atGoalSpeed = false;
    } else {
      outputs.shooterMode = ShooterOutputMode.CLOSED_LOOP;
      outputs.shooterGoalSpeedRadPerSec = shooterGoalSpeedRadPerSec;

      double error = shooterGoalSpeedRadPerSec - inputs.shooterVelocityRadPerSec;

      if (atGoalSpeed) {
        // we were at speed, did we drop velocity suddenly?
        if (Math.abs(error) > 12 && inputs.shooterVelocityRadPerSec < shooterGoalSpeedRadPerSec) {
          atGoalSpeed = isAtGoalSpeedDebouncer.calculate(false);
        } else {
          atGoalSpeed = isAtGoalSpeedDebouncer.calculate(true);
        }
      } else {
        // we are recovering or spinning up
        if (Math.abs(error) < 12 || inputs.shooterVelocityRadPerSec > shooterGoalSpeedRadPerSec) {
          atGoalSpeed = isAtGoalSpeedDebouncer.calculate(true);
        } else {
          atGoalSpeed = isAtGoalSpeedDebouncer.calculate(false);
        }
      }
    }

    io.applyOutputs(outputs);
  }

  @AutoLogOutput
  public int getShotCount() {
    return shotCounter;
  }

  public void setShooterGoalSpeedRadPerSec(double speedRadPerSec) {
    shooterGoalSpeedRadPerSec = speedRadPerSec;
  }

  public Trigger shotDetectedTrigger() {
    return new Trigger(
        () -> {
          boolean fired = shotCounter > prevShotCounter;
          prevShotCounter = shotCounter;
          return fired;
        });
  }

  public Command StopShooter() {
    return runOnce(
        () -> {
          setShooterGoalSpeedRadPerSec(0.0);
        });
  }

  public Command RunShooter() {
    return startEnd(
        () -> setShooterGoalSpeedRadPerSec(ShooterConstants.defaultSpeedRadPerSec),
        () -> setShooterGoalSpeedRadPerSec(0));
  }

  public Command StartShooter() {
    return runOnce(
        () -> {
          setShooterGoalSpeedRadPerSec(ShooterConstants.defaultSpeedRadPerSec);
        });
  }
}
