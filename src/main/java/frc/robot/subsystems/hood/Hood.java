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
import frc.robot.FieldConstants;
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
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/TargetRads", targetPosition);
    Logger.recordOutput("Hood/Currentrads", inputs.positionRad);
  }

  public void setTargetPosition(double position) {
    targetPosition = position;
    outputMode = HoodOutputMode.CLOSED_LOOP;
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

  public Command followHub(Supplier<Pose2d> currentPoseSupplier) {
    return followTargetPosition(currentPoseSupplier, FieldConstants.Hub.centerPose2d);
  }

  public Command followTargetPosition(Supplier<Pose2d> currentPoseSupplier, Pose2d targetPose) {
    return runEnd(
        () -> {
          Pose2d currentPose = currentPoseSupplier.get();
          Logger.recordOutput("Hood/TrackHub/CurrentPose", currentPose);
          Logger.recordOutput("Hood/TrackHub/TargetPose", targetPose);
          setOutputMode(HoodOutputMode.CLOSED_LOOP);
          setTargetPosition(getOptimalRotation(currentPose, targetPose));
        },
        () -> {
          setOutputMode(HoodOutputMode.BRAKE);
        });
  }

  public void setGoalPosition(double goal) {
    setTargetPosition(goal);
  }

  /**
   * https://docs.google.com/spreadsheets/d/1O3aX4AxPTiHVrjRuHEKpu-7YwJeAeOFxhR04oBWCCc4/edit?usp=sharing
   */
  public static double[] solveAnglesRad(double exitVelo, double distance) {
    Logger.recordOutput("Hood/TrackHub/Distance", distance);
    if (exitVelo <= 0.0) {
      return new double[0];
    }
    if (distance <= 1e-6) {
      return new double[0];
    }

    double v2 = exitVelo * exitVelo;
    double v4 = v2 * v2;
    double disc =
        1.0
            - (2.0 * HoodConstants.gravity * HoodConstants.hubHeight) / v2
            - (HoodConstants.gravity * HoodConstants.gravity * distance * distance) / v4;

    if (!Double.isFinite(disc) || disc < 0.0) {
      return new double[0];
    }

    double scale = v2 / (HoodConstants.gravity * distance);
    double t = scale * (1.0 + Math.sqrt(disc));
    double t2 = scale * (1.0 - Math.sqrt(disc));

    double theta1 = (Math.PI / 2.0) - Math.atan(t);
    double theta2 = (Math.PI / 2.0) - Math.atan(t2);

    Logger.recordOutput("Hood/TrackHub/Theta1Raw", theta1);
    Logger.recordOutput("Hood/TrackHub/Theta2Raw", theta2);

    double theta1Out = validateVyDescending(exitVelo, distance, theta1) ? theta1 : Double.NaN;
    double theta2Out = validateVyDescending(exitVelo, distance, theta2) ? theta2 : Double.NaN;
    double[] angles = {
      theta1Out - HoodConstants.ballExitAngle, theta2Out - HoodConstants.ballExitAngle
    };
    Logger.recordOutput("Hood/TrackHub/Theta1Validated", theta1Out);
    Logger.recordOutput("Hood/TrackHub/Theta2Validated", theta2Out);
    Logger.recordOutput("Hood/TrackHub/AnglesOut", angles);

    return angles;
  }

  private static boolean validateVyDescending(double v, double x, double theta) {
    double c = Math.cos(theta);
    if (!Double.isFinite(c) || Math.abs(c) < 1e-9) {
      return false;
    }

    double vy = v * Math.sin(theta) - HoodConstants.gravity * (x / (v * c));
    boolean isDescending = Double.isFinite(vy) && (vy < 0.0);
    return isDescending;
  }

  @AutoLogOutput
  public double getOptimalRotation(Pose2d curPose, Pose2d targetPose) {
    Transform2d transformToTarget = targetPose.minus(curPose);
    double distanceToTarget = transformToTarget.getTranslation().getNorm();

    double[] hoodAnglesRad = solveAnglesRad(HoodConstants.exitVelo, distanceToTarget);

    double currentHoodRad = inputs.positionRad;
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

    double optimalSensorRot = Units.radiansToRotations(closestHoodRad);
    Logger.recordOutput("Hood/TrackHub/OptimalSensorRot", optimalSensorRot);
    return optimalSensorRot;
  }

  public Rotation2d getCurrentHoodRotation() {
    return new Rotation2d(inputs.positionRad);
  }
}
