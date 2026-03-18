// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerIOOutputs;
import frc.robot.subsystems.spindexer.SpindexerIO.SpindexerOutputMode;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends FullSubsystem {

  private SpindexerIO io;
  private final SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();
  public Alert spindexerDisconnectedAlert =
      new Alert("IO Status", "Spindexer disconnected!", AlertType.kError);

  private SpindexerIOOutputs outputs = new SpindexerIOOutputs();
  private double targetSpeed = 0;

  /** Creates a new Spindexer. */
  public Spindexer(SpindexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Spindexer", inputs);
    spindexerDisconnectedAlert.set(!inputs.connected);
  }

  public void setTargetSpeed(double speed) {
    targetSpeed = speed;
  }

  @Override
  public void periodicAfterScheduler() {
    if (targetSpeed == 0) {
      outputs.mode = SpindexerOutputMode.BRAKE;
      outputs.speed = 0;
    } else {
      outputs.mode = SpindexerOutputMode.DUTY_CYCLE;
      outputs.speed = targetSpeed;
    }
    io.applyOutputs(outputs);
  }

  public Command RunSpindexer() {
    return runEnd(
        () -> {
          setTargetSpeed(SpindexerConstants.activeSpeed);
        },
        () -> {
          setTargetSpeed(0);
        });
  }

  public Command StartSpindexer() {
    return run(
        () -> {
          setTargetSpeed(SpindexerConstants.activeSpeed);
        });
  }

  public Command StopSpindexer() {
    return run(
        () -> {
          setTargetSpeed(SpindexerConstants.activeSpeed);
        });
  }
}
