// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final DutyCycleOut shooterActiveDutyCycle =
      new DutyCycleOut(ShooterConstants.shooterActiveVoltageProportion);
  private final DutyCycleOut kickerActiveDutyCycle =
      new DutyCycleOut(ShooterConstants.kickerActiveVoltageProportion);

  private final NeutralOut neutralControl = new NeutralOut();

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command runShooter() {
    return runEnd(
        () -> {
          io.setShooterControl(shooterActiveDutyCycle);
          io.setKickerControl(kickerActiveDutyCycle);
        },
        () -> {
          io.setAllControls(neutralControl);
        });
  }
}
