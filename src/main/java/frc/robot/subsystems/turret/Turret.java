// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FieldConstants;
import frc.robot.subsystems.turret.TurretIO.TurretIOOutputs;
import frc.robot.subsystems.turret.TurretIO.TurretOutputMode;
import frc.robot.util.FullSubsystem;
import java.util.Arrays;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends FullSubsystem {

  private TurretIO io;
  private NeutralOut neutralControlRequest = new NeutralOut();
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();
  public Alert turretDisconnectedAlert =
      new Alert("IO Status", "Turret disconnected!", AlertType.kError);

  private double goalPositionRad = 0;
  private boolean noPositionAvailable = false;
  private TurretOutputMode outputMode = TurretOutputMode.BRAKE;

  /** Creates a new Turret. */
  public Turret(TurretIO io) {
    this.io = io;

    // zero the encoder when the robot is started - turret should begin in the
    // rightmost position
    // io.setEncoderPosition(TurretConstants.encoderStartingPosition * TurretConstants.gearRatio);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    turretDisconnectedAlert.set(!inputs.connected);
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Turret/goalPositionRad", goalPositionRad);
    Logger.recordOutput("Turret/ouputMode", outputMode);
    outputs.mode = outputMode;
    outputs.goalPositionRad = goalPositionRad;
    io.applyOutputs(outputs);
  }

  public void setGoalPositionRad(double positionRad) {
    outputMode = TurretOutputMode.POSITION;
    goalPositionRad = positionRad;
  }

  private void setMotorBrake() {
    outputMode = TurretOutputMode.BRAKE;
  }

  public Command followHub(
      Supplier<Pose2d> currentPoseSupplier, Supplier<ChassisSpeeds> chassisSpeeds) {
    return followTargetPosition(
        currentPoseSupplier, FieldConstants.Hub.centerPose2d, chassisSpeeds);
  }

  public boolean isNoPositionAvailableDuringTracking() {
    return noPositionAvailable && outputMode == TurretOutputMode.POSITION;
  }

  public Trigger noPositionAvailable() {
    return new Trigger(this::isNoPositionAvailableDuringTracking);
  }

  public Command followTargetPosition(
      Supplier<Pose2d> currentPoseSupplier,
      Pose2d targetPose,
      Supplier<ChassisSpeeds> chassisSpeeds) {
    return runEnd(
        () -> {
          OptionalDouble optimalRotation =
              getOptimalRotation(
                  currentPoseSupplier.get(), targetPose, inputs.positionRad, chassisSpeeds.get());
          if (optimalRotation.isEmpty()) {
            noPositionAvailable = true;
          } else {
            setGoalPositionRad(optimalRotation.getAsDouble());
          }
        },
        () -> {
          setMotorBrake();
          ;
        });
  }

  public OptionalDouble findBestTurretAngleRad(double angleRad) {
    double[] candidateRotations = getEquivalentRadians(Rotation2d.fromRadians(angleRad));
    Logger.recordOutput("Turret/candidates", candidateRotations);
    double[] reachableRotations =
        Arrays.stream(candidateRotations).filter(x -> isReachablePosition(x)).toArray();
    Logger.recordOutput("Turret/reachablePositions", reachableRotations);
    if (reachableRotations.length == 0) {
      Logger.recordOutput("Turret/returningEmpty", true);
      return OptionalDouble.empty();
    }
    if (reachableRotations.length == 1) {
      return OptionalDouble.of(reachableRotations[0]);
    }

    double closestRotation = reachableRotations[0];
    Logger.recordOutput("Turret/startingClosest", closestRotation);
    double shortestDistance = Math.abs(inputs.positionRad - reachableRotations[0]);
    for (double r : reachableRotations) {
      double distance = Math.abs(inputs.positionRad - r);
      if (distance < shortestDistance) {
        closestRotation = r;
        shortestDistance = distance;
      }
    }
    return OptionalDouble.of(closestRotation);
  }

  // returns the position (in radians) to spin the motor to in order to follow a
  // target
  public OptionalDouble getOptimalRotation(
      Pose2d curPose, Pose2d targetPose, double currentPositionRad, ChassisSpeeds chassisSpeeds) {
    double[] candidateRotations = getCandidateRotations(curPose, targetPose, chassisSpeeds);
    double[] reachableRotations =
        Arrays.stream(candidateRotations).filter(x -> isReachablePosition(x)).toArray();

    if (reachableRotations.length == 0) return OptionalDouble.empty();
    if (reachableRotations.length == 1) return OptionalDouble.of(reachableRotations[0]);

    double closestRotation = reachableRotations[0];
    double shortestDistance = currentPositionRad - reachableRotations[0];
    for (double r : reachableRotations) {
      double distance = currentPositionRad - r;
      if (distance < shortestDistance) {
        closestRotation = r;
        shortestDistance = distance;
      }
    }
    return OptionalDouble.of(closestRotation);
  }

  public double[] getCandidateRotations(
      Pose2d curPose, Pose2d targetPose, ChassisSpeeds chassisSpeeds) {
    Transform2d transformToTarget = curPose.minus(targetPose);
    Translation2d translationToTarget = transformToTarget.getTranslation().times(-1);

    Rotation2d fieldRelativeAngleToTarget =
        new Rotation2d(translationToTarget.getX(), translationToTarget.getY());
    Rotation2d swerveOrientationAngleToTarget = transformToTarget.getRotation();

    Rotation2d rotationToTarget = fieldRelativeAngleToTarget.minus(swerveOrientationAngleToTarget);

    Logger.recordOutput("Turret/transformToTarget", transformToTarget);
    Logger.recordOutput(
        "Turret/swerveOrientationAngleToTarget", swerveOrientationAngleToTarget.getDegrees());
    Logger.recordOutput(
        "Turret/fieldRelativeAngleToTarget", fieldRelativeAngleToTarget.getDegrees());
    Logger.recordOutput("Turret/translationToTarget", translationToTarget);
    Logger.recordOutput("Turret/rotationToTarget", rotationToTarget.getDegrees());

    return getEquivalentRadians(rotationToTarget);
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
