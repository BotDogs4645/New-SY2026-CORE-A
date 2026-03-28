// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.turret.TurretIO.TurretIOOutputs;
import frc.robot.subsystems.turret.TurretIO.TurretOutputMode;
import frc.robot.util.FullSubsystem;
import java.util.Arrays;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends FullSubsystem {

  private TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();
  public Alert turretDisconnectedAlert =
      new Alert("IO Status", "Turret disconnected!", AlertType.kError);

  private double goalPositionRad = 0;
  private double appliedOffsetRad = 0;
  private boolean noPositionAvailable = false;
  private TurretOutputMode outputMode = TurretOutputMode.BRAKE;
  private boolean atGoalPosition = false;

  /** Creates a new Turret. */
  public Turret(TurretIO io) {
    this.io = io;

    // zero the encoder when the robot is started - turret should begin in the
    // rightmost position
    // io.setEncoderPosition(TurretConstants.encoderStartingPosition *
    // TurretConstants.gearRatio);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    turretDisconnectedAlert.set(!inputs.connected);
  }

  @Override
  public void periodicAfterScheduler() {
    outputs.mode = outputMode;
    outputs.goalPositionRad = goalPositionRad + appliedOffsetRad;
    io.applyOutputs(outputs);
    if (outputMode == TurretOutputMode.POSITION) {
      double positionToleranceRad = Units.degreesToRadians(4);
      atGoalPosition =
          Math.abs(inputs.realPositionRad - (goalPositionRad + appliedOffsetRad))
              < positionToleranceRad;
    } else {
      atGoalPosition = false;
    }
  }

  public void setGoalPositionRad(double positionRad) {
    outputMode = TurretOutputMode.POSITION;
    goalPositionRad = positionRad;
  }

  public enum Direction {
    CW,
    CCW
  }

  public void addOffset(Direction direction, double angleDegrees) {
    if (direction == Direction.CW) {
      appliedOffsetRad -= Units.degreesToRadians(angleDegrees);
    } else {
      appliedOffsetRad += Units.degreesToRadians(angleDegrees);
    }
  }

  public void setOffset(double angleDegrees) {
    appliedOffsetRad = Units.degreesToRadians(angleDegrees);
  }

  public Command AddTurretOffset(Direction direction, BooleanSupplier staticAiming) {
    return Commands.run(
        () -> {
          if (staticAiming.getAsBoolean()) {
            addOffset(direction, TurretConstants.staticOverrideOffsetPeriodicDegrees);
          } else {
            addOffset(direction, TurretConstants.offsetPeriodicDegrees);
          }
        });
  }

  public Command ResetTurretOffset() {
    return Commands.runOnce(() -> setOffset(0));
  }

  private void setMotorBrake() {
    outputMode = TurretOutputMode.BRAKE;
  }

  @AutoLogOutput
  public boolean isAtGoalPosition() {
    return atGoalPosition;
  }

  public boolean isNoPositionAvailableDuringTracking() {
    return noPositionAvailable && outputMode == TurretOutputMode.POSITION;
  }

  public Trigger noPositionAvailable() {
    return new Trigger(this::isNoPositionAvailableDuringTracking);
  }

  public OptionalDouble findBestTurretAngleRad(double angleRad) {
    double[] candidateRotations = getEquivalentRadians(Rotation2d.fromRadians(angleRad));
    Logger.recordOutput("Turret/AngleFinder/candidates", candidateRotations);
    double[] reachableRotations =
        Arrays.stream(candidateRotations).filter(x -> isReachablePosition(x)).toArray();
    Logger.recordOutput("Turret/AngleFinder/reachablePositions", reachableRotations);
    if (reachableRotations.length == 0) {
      Logger.recordOutput("Turret/AngleFinder/returningEmpty", true);
      return OptionalDouble.empty();
    }
    if (reachableRotations.length == 1) {
      return OptionalDouble.of(reachableRotations[0]);
    }

    double closestRotation = reachableRotations[0];
    Logger.recordOutput("Turret/AngleFinder/startingClosest", closestRotation);
    double shortestDistance = Math.abs(inputs.realPositionRad - reachableRotations[0]);
    for (double r : reachableRotations) {
      double distance = Math.abs(inputs.realPositionRad - r);
      if (distance < shortestDistance) {
        closestRotation = r;
        shortestDistance = distance;
      }
    }
    return OptionalDouble.of(closestRotation);
  }

  /**
   * Returns all possible alterate angles for a given Rotation2D on the range -2pi to 2pi, including
   * the original angle.
   *
   * <p>Notes: - All angles are in radians. - Equality to zero will trigger a three-value result.
   *
   * @param rotation the Rotation2d whose radian equivalents are desired
   * @return an array of doubles containing equivalent radian representations: length 3 when
   *     rotation is exactly 0 (-2π, 0, +2π), otherwise length 2
   */
  public static double[] getEquivalentRadians(Rotation2d rotation) {
    double primary = rotation.getRadians();

    if (primary == 0) {
      return new double[] {-(2 * Math.PI), 0, 2 * Math.PI};
    }

    double other = primary >= 0 ? primary - (2 * Math.PI) : primary + (2 * Math.PI);
    return new double[] {primary, other};
  }

  /**
   * Determines whether a given turret angle (in radians) is allowed based on the configured hard
   * limits.
   *
   * <p>This method returns true only when the provided angle is strictly greater than both
   * TurretConstants.hardReverseLimit and TurretConstants.hardForwardLimit. Values equal to either
   * hard limit are considered invalid and will return false.
   *
   * @param angleRad the candidate turret angle, in radians
   * @return true if angleRad is strictly greater than both hardReverseLimit and hardForwardLimit;
   *     false otherwise
   */
  public static boolean isReachablePosition(double angleRad) {
    return angleRad > TurretConstants.hardReverseLimit
        && angleRad < TurretConstants.hardForwardLimit;
  }

  public void resetEncoder() {
    io.setEncoderPosition(0);
  }
}
