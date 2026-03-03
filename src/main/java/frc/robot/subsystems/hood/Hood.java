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
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/TargetRot", targetPosition);
    Logger.recordOutput("Hood/CurrentRot", Units.radiansToRotations(inputs.positionRad));
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

  public Command followHub(Supplier<Pose2d> currentPoseSupplier) {
    return followTargetPosition(currentPoseSupplier, FieldPoses.HUB);
  }

  public Command followTargetPosition(Supplier<Pose2d> currentPoseSupplier, Pose2d targetPose) {
    return runEnd(
        () -> {
          setOutputMode(HoodOutputMode.CLOSED_LOOP);
          setTargetPosition(getOptimalRotation(currentPoseSupplier.get(), targetPose));
        },
        () -> setOutputMode(HoodOutputMode.BRAKE));
  }

  /**
   * https://docs.google.com/spreadsheets/d/1O3aX4AxPTiHVrjRuHEKpu-7YwJeAeOFxhR04oBWCCc4/edit?usp=sharing
   */
  public static double[] solveAnglesRad(double exitVelo, double distance) {
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

    double root = Math.sqrt(disc);

    double scale = v2 / (HoodConstants.gravity * distance);
    double t = scale * (1.0 + root);
    double t2 = scale * (1.0 - root);

    double a = Math.atan(t);
    double a2 = Math.atan(t2);

    double theta1 = (Math.PI / 2.0) - a;
    double theta2 = (Math.PI / 2.0) - a2;

    double theta1Out = validateVyDescending(exitVelo, distance, theta1) ? theta1 : Double.NaN;
    double theta2Out = validateVyDescending(exitVelo, distance, theta2) ? theta2 : Double.NaN;
    double[] angles = {
      theta1Out - HoodConstants.ballExitAngle, theta2Out - HoodConstants.ballExitAngle
    };

    return angles;
  }

  private static boolean validateVyDescending(double v, double x, double theta) {
    double c = Math.cos(theta);
    if (!Double.isFinite(c) || Math.abs(c) < 1e-9) return false;

    double vy = v * Math.sin(theta) - HoodConstants.gravity * (x / (v * c));
    return Double.isFinite(vy) && (vy < 0.0);
  }

  @AutoLogOutput
  public double getOptimalRotation(Pose2d curPose, Pose2d targetPose) {
    Transform2d transformToTarget = targetPose.minus(curPose);
    double distanceToTarget = transformToTarget.getTranslation().getNorm();

    double[] hoodAnglesRad = solveAnglesRad(HoodConstants.exitVelo, distanceToTarget);

    double currentHoodRad = sensorRadToHoodRad(inputs.positionRad);

    double closestHoodRad =
        Arrays.stream(hoodAnglesRad)
            .filter(Double::isFinite)
            .boxed()
            .min(Comparator.comparingDouble(a -> Math.abs(a - currentHoodRad)))
            .orElse(Double.NaN);

    if (!Double.isFinite(closestHoodRad)) {
      return targetPosition;
    }

    return hoodRadToSensorRot(closestHoodRad);
  }

  private double sensorRadToHoodRad(double sensorRad) {
    return sensorRad / HoodConstants.gearRatio;
  }

  private double hoodRadToSensorRot(double hoodRad) {
    double hoodRot = Units.radiansToRotations(hoodRad);
    return hoodRot * HoodConstants.gearRatio + HoodConstants.offset;
  }

  public double convertToTurretPosition(Rotation2d angle) {
    return angle.getRotations() * HoodConstants.gearRatio;
  }
}
