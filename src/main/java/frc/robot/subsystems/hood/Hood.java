// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldPoses;
import frc.robot.subsystems.hood.HoodIO.HoodIOOutputs;
import frc.robot.subsystems.hood.HoodIO.HoodOutputMode;
import frc.robot.util.FullSubsystem;
import java.util.Arrays;
import java.util.Comparator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
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
    Logger.recordOutput("Hood/Init/TargetRot", targetPosition);
    Logger.recordOutput("Hood/Init/OutputMode", outputMode.name());
    Logger.recordOutput("Hood/Init/IsAtTarget", isAtTargetPosition);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/TargetRot", targetPosition);
    Logger.recordOutput("Hood/CurrentRot", Units.radiansToRotations(inputs.positionRad));
    Logger.recordOutput("Hood/Periodic/CurrentPositionRad", inputs.positionRad);
    Logger.recordOutput(
        "Hood/Periodic/CurrentPositionHoodRad", sensorRadToHoodRad(inputs.positionRad));
    Logger.recordOutput("Hood/Periodic/OutputMode", outputMode.name());
    Logger.recordOutput(
        "Hood/Periodic/TargetErrorRot",
        targetPosition - Units.radiansToRotations(inputs.positionRad));
    Logger.recordOutput(
        "Hood/Periodic/TargetErrorAbsRot",
        Math.abs(targetPosition - Units.radiansToRotations(inputs.positionRad)));
  }

  public void setTargetPosition(double position) {
    Logger.recordOutput("Hood/SetTargetPosition/PreviousTargetRot", targetPosition);
    Logger.recordOutput("Hood/SetTargetPosition/NewTargetRot", position);
    targetPosition = position;
    Logger.recordOutput("Hood/SetTargetPosition/AppliedTargetRot", targetPosition);
  }

  public void setOutputMode(HoodOutputMode mode) {
    Logger.recordOutput("Hood/SetOutputMode/PreviousMode", outputMode.name());
    Logger.recordOutput("Hood/SetOutputMode/NewMode", mode.name());
    outputMode = mode;
    Logger.recordOutput("Hood/SetOutputMode/AppliedMode", outputMode.name());
  }

  @Override
  public void periodicAfterScheduler() {
    outputs.mode = outputMode;
    outputs.targetPosition = targetPosition;
    double currentPosition = Units.radiansToRotations(inputs.positionRad);
    Logger.recordOutput("Hood/PeriodicAfterScheduler/OutputMode", outputs.mode.name());
    Logger.recordOutput("Hood/PeriodicAfterScheduler/OutputTargetRot", outputs.targetPosition);
    Logger.recordOutput("Hood/PeriodicAfterScheduler/CurrentRot", currentPosition);
    Logger.recordOutput("Hood/PeriodicAfterScheduler/CurrentRad", inputs.positionRad);
    Logger.recordOutput(
        "Hood/PeriodicAfterScheduler/CurrentHoodRad", sensorRadToHoodRad(inputs.positionRad));
    Logger.recordOutput(
        "Hood/PeriodicAfterScheduler/ErrorRot", outputs.targetPosition - currentPosition);
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

  public Command followHub(Supplier<Pose2d> currentPoseSupplier) {
    return followTargetPosition(currentPoseSupplier, FieldPoses.HUB);
  }

  public Command followTargetPosition(Supplier<Pose2d> currentPoseSupplier, Pose2d targetPose) {
    return runEnd(
        () -> {
          Pose2d currentPose = currentPoseSupplier.get();
          Logger.recordOutput("Hood/TrackHub/FollowTargetPosition/LoopActive", true);
          Logger.recordOutput("Hood/TrackHub/FollowTargetPosition/CurrentPose", currentPose);
          Logger.recordOutput("Hood/TrackHub/FollowTargetPosition/TargetPose", targetPose);
          setOutputMode(HoodOutputMode.CLOSED_LOOP);
          setTargetPosition(getOptimalRotation(currentPose, targetPose));
        },
        () -> {
          Logger.recordOutput("Hood/TrackHub/FollowTargetPosition/LoopActive", false);
          setOutputMode(HoodOutputMode.BRAKE);
        });
  }

  /**
   * https://docs.google.com/spreadsheets/d/1O3aX4AxPTiHVrjRuHEKpu-7YwJeAeOFxhR04oBWCCc4/edit?usp=sharing
   */
  public static double[] solveAnglesRad(double exitVelo, double distance) {
    Logger.recordOutput("Hood/Ballistics/ExitVelocity", exitVelo);
    Logger.recordOutput("Hood/Ballistics/Distance", distance);
    if (exitVelo <= 0.0) {
      Logger.recordOutput("Hood/Ballistics/InvalidExitVelocity", true);
      return new double[0];
    }
    if (distance <= 1e-6) {
      Logger.recordOutput("Hood/Ballistics/InvalidDistance", true);
      return new double[0];
    }

    double v2 = exitVelo * exitVelo;
    double v4 = v2 * v2;
    double disc =
        1.0
            - (2.0 * HoodConstants.gravity * HoodConstants.hubHeight) / v2
            - (HoodConstants.gravity * HoodConstants.gravity * distance * distance) / v4;
    Logger.recordOutput("Hood/Ballistics/Discriminant", disc);

    if (!Double.isFinite(disc) || disc < 0.0) {
      Logger.recordOutput("Hood/Ballistics/InvalidDiscriminant", true);
      return new double[0];
    }

    double root = Math.sqrt(disc);

    double scale = v2 / (HoodConstants.gravity * distance);
    double t = scale * (1.0 + root);
    double t2 = scale * (1.0 - root);
    Logger.recordOutput("Hood/Ballistics/T1", t);
    Logger.recordOutput("Hood/Ballistics/T2", t2);

    double a = Math.atan(t);
    double a2 = Math.atan(t2);

    double theta1 = (Math.PI / 2.0) - a;
    double theta2 = (Math.PI / 2.0) - a2;
    Logger.recordOutput("Hood/Ballistics/Theta1Raw", theta1);
    Logger.recordOutput("Hood/Ballistics/Theta2Raw", theta2);

    double theta1Out = validateVyDescending(exitVelo, distance, theta1) ? theta1 : Double.NaN;
    double theta2Out = validateVyDescending(exitVelo, distance, theta2) ? theta2 : Double.NaN;
    double[] angles = {
      theta1Out - HoodConstants.ballExitAngle, theta2Out - HoodConstants.ballExitAngle
    };
    Logger.recordOutput("Hood/Ballistics/Theta1Validated", theta1Out);
    Logger.recordOutput("Hood/Ballistics/Theta2Validated", theta2Out);
    Logger.recordOutput("Hood/Ballistics/AnglesOut", angles);

    return angles;
  }

  private static boolean validateVyDescending(double v, double x, double theta) {
    double c = Math.cos(theta);
    if (!Double.isFinite(c) || Math.abs(c) < 1e-9) {
      Logger.recordOutput("Hood/Ballistics/Validate/CosInvalid", true);
      return false;
    }

    double vy = v * Math.sin(theta) - HoodConstants.gravity * (x / (v * c));
    boolean isDescending = Double.isFinite(vy) && (vy < 0.0);
    Logger.recordOutput("Hood/Ballistics/Validate/Vy", vy);
    Logger.recordOutput("Hood/Ballistics/Validate/IsDescending", isDescending);
    return isDescending;
  }

  @AutoLogOutput
  public double getOptimalRotation(Pose2d curPose, Pose2d targetPose) {
    Transform2d transformToTarget = targetPose.minus(curPose);
    double distanceToTarget = transformToTarget.getTranslation().getNorm();
    Logger.recordOutput("Hood/TrackHub/CurrentPose", curPose);
    Logger.recordOutput("Hood/TrackHub/TargetPose", targetPose);
    Logger.recordOutput("Hood/TrackHub/Transform", transformToTarget);
    Logger.recordOutput("Hood/TrackHub/DistanceToTarget", distanceToTarget);

    double[] hoodAnglesRad = solveAnglesRad(HoodConstants.exitVelo, distanceToTarget);
    Logger.recordOutput("Hood/TrackHub/SolvedAnglesRad", hoodAnglesRad);

    double currentHoodRad = sensorRadToHoodRad(inputs.positionRad);
    Logger.recordOutput("Hood/TrackHub/CurrentSensorRad", inputs.positionRad);
    Logger.recordOutput("Hood/TrackHub/CurrentHoodRad", currentHoodRad);

    double closestHoodRad =
        Arrays.stream(hoodAnglesRad)
            .filter(Double::isFinite)
            .boxed()
            .min(Comparator.comparingDouble(a -> Math.abs(a - currentHoodRad)))
            .orElse(Double.NaN);
    Logger.recordOutput("Hood/TrackHub/ClosestHoodRad", closestHoodRad);

    if (!Double.isFinite(closestHoodRad)) {
      Logger.recordOutput("Hood/TrackHub/NoValidAngle", true);
      return targetPosition;
    }

    double optimalSensorRot = hoodRadToSensorRot(closestHoodRad);
    Logger.recordOutput("Hood/TrackHub/OptimalSensorRot", optimalSensorRot);
    return optimalSensorRot;
  }

  private double sensorRadToHoodRad(double sensorRad) {
    double hoodRad = sensorRad / HoodConstants.gearRatio;
    Logger.recordOutput("Hood/Convert/SensorRadToHoodRad/OutputHoodRad", hoodRad);
    return hoodRad;
  }

  private double hoodRadToSensorRot(double hoodRad) {
    double hoodRot = Units.radiansToRotations(hoodRad);
    double sensorRot = hoodRot * HoodConstants.gearRatio + HoodConstants.offset;
    Logger.recordOutput("Hood/Convert/HoodRadToSensorRot", sensorRot);
    return sensorRot;
  }

  public double convertToTurretPosition(Rotation2d angle) {
    double turretPosition = angle.getRotations() * HoodConstants.gearRatio;
    Logger.recordOutput("Hood/Convert/ToTurretPosition", turretPosition);
    return turretPosition;
  }
}
