// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldPoses;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {

  private TurretIO io;
  private NeutralOut neutralControlRequest = new NeutralOut();
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  /** Creates a new Turret. */
  public Turret(TurretIO io) {
    this.io = io;

    // zero the encoder when the robot is started - turret should begin in the
    // rightmost position
    io.setEncoderPosition(TurretConstants.encoderStartingPosition);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public Command followHub(Supplier<Pose2d> currentPoseSupplier) {
    return followTargetPosition(currentPoseSupplier, FieldPoses.HUB);
  }

  public Command followTargetPosition(Supplier<Pose2d> currentPoseSupplier, Pose2d targetPose) {
    MotionMagicVoltage controlRequest =
        new MotionMagicVoltage(getOptimalRotation(currentPoseSupplier.get(), targetPose));
    return runEnd(
        () -> {
          io.setMotorControl(
              controlRequest.withPosition(
                  getOptimalRotation(currentPoseSupplier.get(), targetPose)));
          Logger.recordOutput(
              "Turret/optimalPosition", getOptimalRotation(currentPoseSupplier.get(), targetPose));
        },
        () -> {
          io.setMotorControl(neutralControlRequest);
        });
  }

  // returns the position (in rotations) to spin the motor to in order to follow a
  // target
  @AutoLogOutput
  public double getOptimalRotation(Pose2d curPose, Pose2d targetPose) {
    Transform2d transformToTarget = curPose.minus(targetPose);
    Logger.recordOutput("Turret/transformToTarget", transformToTarget);
    Translation2d translationToTarget = transformToTarget.getTranslation().times(-1);
    Logger.recordOutput("Turret/translationToTarget", translationToTarget);

    Rotation2d translationDirectionToTarget =
        new Rotation2d(translationToTarget.getX(), translationToTarget.getY());
    Logger.recordOutput(
        "Turret/translationDirectionToTarget", translationDirectionToTarget.getDegrees());
    Rotation2d tranformAngle = transformToTarget.getRotation();
    Logger.recordOutput("Turret/transformAngle", tranformAngle.getDegrees());

    Rotation2d rotationToTarget = translationDirectionToTarget.minus(tranformAngle);
    Logger.recordOutput("Turret/rotationToTarget", rotationToTarget.getDegrees());

    return convertToTurretPosition(rotationToTarget);
  }

  public double convertToTurretPosition(Rotation2d angle) {
    Rotation2d withInitialPos =
        angle.minus(new Rotation2d(TurretConstants.physicalStartingPosition));
    return withInitialPos.getRotations() * TurretConstants.gearRatio;
  }

  public void resetEncoder() {
    io.setEncoderPosition(0);
  }
}
