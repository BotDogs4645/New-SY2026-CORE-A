// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodIO.HoodIOOutputs;
import frc.robot.subsystems.hood.HoodIO.HoodOutputMode;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.Logger;

public class Hood extends FullSubsystem {

  private HoodIO io;
  private NeutralOut neutralControlRequest = new NeutralOut();
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();
  private double targetPosition = 0;
  private HoodOutputMode outputMode = HoodOutputMode.BRAKE;
  private boolean isAtTargetPosition = false;

  /** Creates a new Hood. */
  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public void setTargetPosition(double position) {
    targetPosition = position;
  }

  public void setOutputMode(HoodOutputMode mode) {
    outputMode = mode;
  }

  @Override
  public void periodicAfterScheduler() {
    outputs.mode = outputMode;
    outputs.targetPosition = targetPosition;
    double currentPosition = Units.radiansToRotations(inputs.positionRad);
    if (Math.abs(currentPosition - targetPosition) < 0.004) {
      isAtTargetPosition = true;
    } else {
      isAtTargetPosition = false;
    }
    Logger.recordOutput("Hood/isAtTarget", isAtTargetPosition);
    io.applyOutputs(outputs);
  }

  public Command raiseHood() {
    return runOnce(
        () -> {
          setTargetPosition(0.08);
          setOutputMode(HoodOutputMode.CLOSED_LOOP);
        });
  }

  public Command lowerHood() {
    return runOnce(
        () -> {
          setTargetPosition(0);
          setOutputMode(HoodOutputMode.CLOSED_LOOP);
        });
  }
}
