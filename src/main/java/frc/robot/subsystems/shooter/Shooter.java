// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOOutputs;
import frc.robot.subsystems.shooter.ShooterIO.ShooterOutputMode;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.Logger;

public class Shooter extends FullSubsystem {

  private ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterIOOutputs outputs = new ShooterIOOutputs();

  private double goalSpeedRadPerSec = 0.0;
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
    if (goalSpeedRadPerSec == 0.0) {
      outputs.mode = ShooterOutputMode.COAST;
      outputs.goalSpeedRadPerSec = 0.0;
      atGoalSpeed = false;
    } else {
      outputs.mode = ShooterOutputMode.CLOSED_LOOP;
      outputs.goalSpeedRadPerSec = goalSpeedRadPerSec;

      double error = goalSpeedRadPerSec - inputs.shooterVelocityRadPerSec;

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

    io.applyOutputs(outputs);
  }

  public void setGoalSpeedRadPerSec(double speedRadPerSec) {
    goalSpeedRadPerSec = speedRadPerSec;
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
          setGoalSpeedRadPerSec(0.0);
        });
  }
}
