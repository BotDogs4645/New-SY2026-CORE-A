// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOOutputs;
import frc.robot.subsystems.shooter.ShooterIO.ShooterOutputMode;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends FullSubsystem {

  private ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOOutputs outputs = new ShooterIOOutputs();

  private double shooterGoalSpeedRadPerSec = 0.0;
  private double kickerGoalSpeedRadPerSec = 0.0;
  private boolean atGoalSpeed = false;
  private int prevShotCounter = 0;
  private int shotCounter = 0;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Shooter/shooterVelocityRadPerSec", shooterGoalSpeedRadPerSec);
    Logger.recordOutput("Shooter/kickerVelocityRadPerSec", kickerGoalSpeedRadPerSec);
    if (shooterGoalSpeedRadPerSec == 0.0) {
      outputs.shooterMode = ShooterOutputMode.BRAKE;
      outputs.shooterGoalSpeedRadPerSec = 0.0;
      atGoalSpeed = false;
    } else {
      outputs.shooterMode = ShooterOutputMode.CLOSED_LOOP;
      outputs.shooterGoalSpeedRadPerSec = shooterGoalSpeedRadPerSec;

      double error = shooterGoalSpeedRadPerSec - inputs.shooterVelocityRadPerSec;

      if (atGoalSpeed) {
        // we were at speed, did we drop velocity suddenly?
        if (error > 10) {
          shotCounter++;
          atGoalSpeed = false;
        }
      } else {
        // we are recovering or spinning up
        if (Math.abs(error) < 10) {
          atGoalSpeed = true;
        }
      }
    }

    if (kickerGoalSpeedRadPerSec == 0.0) {
      outputs.kickerMode = ShooterOutputMode.BRAKE;
      outputs.kickerGoalSpeedRadPerSec = 0.0;
      atGoalSpeed = false;
    } else {
      outputs.kickerMode = ShooterOutputMode.CLOSED_LOOP;
      outputs.kickerGoalSpeedRadPerSec = kickerGoalSpeedRadPerSec;
    }

    io.applyOutputs(outputs);
  }

  public void setShooterGoalSpeedRadPerSec(double speedRadPerSec) {
    shooterGoalSpeedRadPerSec = speedRadPerSec;
  }

  public void setKickerGoalSpeedRadPerSec(double speedRadPerSec) {
    kickerGoalSpeedRadPerSec = speedRadPerSec;
  }

  public Trigger shotDetectedTrigger() {
    return new Trigger(
        () -> {
          boolean fired = shotCounter > prevShotCounter;
          prevShotCounter = shotCounter;
          return fired;
        });
  }

  public Command stopShooter() {
    return runOnce(
        () -> {
          setKickerGoalSpeedRadPerSec(0.0);
          setShooterGoalSpeedRadPerSec(0.0);
        });
  }

  public Command runShooter() {
    return Commands.sequence(
        runOnce(() -> setShooterGoalSpeedRadPerSec(300)),
        Commands.waitSeconds(0.2),
        startEnd(
            () -> setKickerGoalSpeedRadPerSec(300),
            () -> {
              setKickerGoalSpeedRadPerSec(0.0);
              setShooterGoalSpeedRadPerSec(0.0);
            }));
  }
}
